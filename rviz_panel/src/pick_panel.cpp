// RViz panel: overhead camera with detections (/vision/image) and the board variants that can be picked now
// (/vision/boards). Click a variant button, or a board in the camera image = supervised pick and place.
// Wrist D405 color image (pixi run wrist) appears below the overhead view while it publishes.
//
// Runs robot/pick_one.py --place --variant X as a child process, so RViz must run in the robot workspace env
// (robot/rviz.sh does that). Its output is shown here; its ENTER prompts become the Confirm button.
// /pick_panel/request (std_msgs/String) "VARa" | "VARb" | "VARc" | "confirm" | "stop" = the buttons, for scripts and a
// later chat layer. A "confirm" from the topic is refused unless pick_one reported MOCK hardware: on the real robot a
// human presses the button. /pick_panel/log (std_msgs/String): every output line of pick_one.
#include <signal.h>

#include <QDateTime>
#include <QEvent>
#include <QMouseEvent>
#include <QPolygonF>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProcess>
#include <QPushButton>
#include <QSet>
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <hk_fr5_interfaces/msg/gripper_state.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace vision1_rviz
{
const QStringList VARIANTS = {"VARa", "VARb", "VARc"};
constexpr double STALE_S = 2.0;  // no /vision/boards for this long -> buttons off

class PickPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PickPanel(QWidget * parent = nullptr) : Panel(parent)
  {
    auto * outer = new QVBoxLayout(this);
    auto * scroll = new QScrollArea;
    auto * content = new QWidget;
    auto * lay = new QVBoxLayout(content);
    scroll->setWidget(content);
    scroll->setWidgetResizable(true);
    outer->addWidget(scroll);
    setMinimumWidth(560);
    setStyleSheet("QLabel,QLineEdit,QPushButton {font-size:14px;} QPlainTextEdit {font-size:12px;}");
    status_ = new QLabel;  // live health of everything the demo needs (refresh())
    status_->setWordWrap(true);
    status_->setStyleSheet("font-size:13px");
    lay->addWidget(status_);
    cam_ = new QLabel("camera: waiting for /vision/image");
    cam_->setAlignment(Qt::AlignCenter);
    cam_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);  // no resize loop from the pixmap
    cam_->setMinimumSize(480, 270);
    cam_->setStyleSheet("background:black;color:white");
    cam_->setToolTip("click a board = pick and place it");
    cam_->installEventFilter(this);
    lay->addWidget(cam_, 3);
    wrist_ = new QLabel;
    wrist_->setAlignment(Qt::AlignCenter);
    wrist_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    wrist_->setMinimumSize(320, 180);
    wrist_->setStyleSheet("background:black");
    wrist_->setToolTip("wrist camera (D405)");
    wrist_->installEventFilter(this);
    wrist_->hide();  // shown while wrist images arrive
    lay->addWidget(wrist_, 3);
    vision_ = new QLabel("vision: waiting for /vision/boards (pixi run vision)");
    vision_->setWordWrap(true);
    vision_->setMinimumHeight(60);
    lay->addWidget(vision_);
    lay->addWidget(new QLabel("<b>Click a variant (or a board in the image) to pick and place it:</b>"));
    auto * row = new QHBoxLayout;
    for (const auto & v : VARIANTS) {
      auto * b = new QPushButton(v);
      b->setMinimumHeight(78);
      b->setEnabled(false);
      connect(b, &QPushButton::clicked, this, [this, v] {start(v, "button");});
      row->addWidget(b);
      buttons_[v] = b;
    }
    lay->addLayout(row);
    auto * args_row = new QHBoxLayout;
    args_row->addWidget(new QLabel("extra args"));
    args_ = new QLineEdit;
    args_->setPlaceholderText("e.g. --speed 0.2 --temp -0.25 0.10");
    args_row->addWidget(args_);
    lay->addLayout(args_row);
    run_ = new QLabel("robot: idle");
    run_->setWordWrap(true);
    lay->addWidget(run_);
    auto * ctl = new QHBoxLayout;
    confirm_ = new QPushButton("Confirm (ENTER)");
    stop_ = new QPushButton("Stop script");
    stop_->setToolTip("Cancel the active pick action. This is not the emergency stop; use the pendant for emergencies.");
    for (auto * b : {confirm_, stop_}) {
      b->setEnabled(false);
      b->setMinimumHeight(50);
      ctl->addWidget(b);
    }
    connect(confirm_, &QPushButton::clicked, this, [this] {confirm("button");});
    connect(stop_, &QPushButton::clicked, this, &PickPanel::stop);
    lay->addLayout(ctl);
    log_ = new QPlainTextEdit;
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(2000);
    log_->setFont(QFont("Monospace", 9));
    log_->setMaximumHeight(110);
    lay->addWidget(log_, 1);
    chat_ = new QPlainTextEdit;
    chat_->setReadOnly(true);
    chat_->setMaximumBlockCount(200);
    chat_->setFixedHeight(110);
    chat_->setPlaceholderText("Local assistant: start the conversation bridge and model server.");
    outer->addWidget(chat_);
    chat_status_ = new QLabel("Assistant: offline");
    outer->addWidget(chat_status_);
    batch_status_ = new QLabel("Batch: idle");
    batch_status_->setWordWrap(true);
    outer->addWidget(batch_status_);
    auto * chat_row = new QHBoxLayout;
    chat_input_ = new QLineEdit;
    chat_input_->setPlaceholderText("move two variant B to drop off b / status / pause / stop / what do you see?");
    auto * send = new QPushButton("Send");
    auto * talk = new QPushButton("Hold to talk");
    auto * clear = new QPushButton("Clear chat");
    auto * cancel_batch = new QPushButton("Stop robot (main controller)");
    cancel_batch->setToolTip("Main controller stop: smooth stop now, then paused. Not an emergency stop.");
    chat_row->addWidget(chat_input_, 1);
    chat_row->addWidget(send);
    chat_row->addWidget(talk);
    chat_row->addWidget(clear);
    outer->addLayout(chat_row);
    outer->addWidget(cancel_batch);
    auto submit = [this] {
        if (!chat_input_->text().trimmed().isEmpty() && chat_pub_) {
          std_msgs::msg::String m; m.data = chat_input_->text().toStdString();
          chat_pub_->publish(m); chat_input_->clear();
        }
      };
    connect(send, &QPushButton::clicked, this, submit);
    connect(chat_input_, &QLineEdit::returnPressed, this, submit);
    connect(talk, &QPushButton::pressed, this, [this] {chatControl("ptt_start");});
    connect(talk, &QPushButton::released, this, [this] {chatControl("ptt_stop");});
    connect(clear, &QPushButton::clicked, this, [this] {chatControl("clear");});
    connect(cancel_batch, &QPushButton::clicked, this, [this] {chatControl("stop");});
    auto * t = new QTimer(this);
    connect(t, &QTimer::timeout, this, &PickPanel::refresh);
    t->start(500);
    if (qEnvironmentVariableIsSet("CONVERSATION_SCREENSHOT")) {
      QTimer::singleShot(8000, this, [this] {window()->grab().save(qEnvironmentVariable("CONVERSATION_SCREENSHOT"));});
    }
  }

  ~PickPanel() override
  {
    if (running()) {  // closing RViz stops the script like Ctrl+C
      ::kill(proc_->processId(), SIGINT);
      if (!proc_->waitForFinished(2000)) {
        proc_->kill();
        proc_->waitForFinished(1000);
      }
    }
  }

  void onInitialize() override
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    // queued: runs on the Qt thread whichever thread spins the node
    boards_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/vision/boards", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
        QMetaObject::invokeMethod(
          this, [this, s = QString::fromStdString(m->data)] {onBoards(s);}, Qt::QueuedConnection);
      });
    request_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/pick_panel/request", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
        QMetaObject::invokeMethod(
          this, [this, s = QString::fromStdString(m->data).trimmed()] {onRequest(s);}, Qt::QueuedConnection);
      });
    // best effort: matches reliable and best-effort publishers (the RealSense driver's default differs by version)
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/vision/image", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        QImage img = toQImage(*m);
        if (!img.isNull()) {QMetaObject::invokeMethod(this, [this, img] {onImage(img);}, Qt::QueuedConnection);}
      });
    wrist_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/wrist/wrist_camera/color/image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        QImage img = toQImage(*m);
        if (!img.isNull()) {QMetaObject::invokeMethod(this, [this, img] {onWrist(img);}, Qt::QueuedConnection);}
      });
    joints_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr) {
        QMetaObject::invokeMethod(this, [this] {joints_ms_ = QDateTime::currentMSecsSinceEpoch();}, Qt::QueuedConnection);
      });
    desc_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::String::ConstSharedPtr m) {
        const bool mock = m->data.find("mock_components") != std::string::npos;
        QMetaObject::invokeMethod(this, [this, mock] {hw_kind_ = mock ? "SIMULATION" : "REAL FR5";}, Qt::QueuedConnection);
      });
    gripper_sub_ = node_->create_subscription<hk_fr5_interfaces::msg::GripperState>(
      "/gripper/state", rclcpp::QoS(1).transient_local(), [this](hk_fr5_interfaces::msg::GripperState::ConstSharedPtr m) {
        QMetaObject::invokeMethod(this, [this, s = QString::fromStdString(m->state)] {gripper_ = s;}, Qt::QueuedConnection);
      });
    log_pub_ = node_->create_publisher<std_msgs::msg::String>("/pick_panel/log", 50);
    state_pub_ = node_->create_publisher<std_msgs::msg::String>("/pick_panel/state", 10);
    chat_pub_ = node_->create_publisher<std_msgs::msg::String>("/conversation/input", 10);
    chat_control_pub_ = node_->create_publisher<std_msgs::msg::String>("/conversation/control", 10);
    batch_sub_ = node_->create_subscription<std_msgs::msg::String>("/conversation/batch_state", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        QMetaObject::invokeMethod(this, [this, s = QString::fromStdString(m->data)] {
          const auto o = QJsonDocument::fromJson(s.toUtf8()).object();
          batch_status_->setText(QString("Batch: %1 — %2 / %3 completed. %4")
            .arg(o["phase"].toString()).arg(o["completed"].toInt()).arg(o["total"].toInt()).arg(o["reason"].toString()));
        }, Qt::QueuedConnection);
      });
    chat_sub_ = node_->create_subscription<std_msgs::msg::String>("/conversation/output", 20,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        QMetaObject::invokeMethod(this, [this, s = QString::fromStdString(m->data)] {
          const auto o = QJsonDocument::fromJson(s.toUtf8()).object();
          if (o["phase"].toString() == "clear") {chat_->clear();}
          chat_->appendPlainText(o["role"].toString() + ": " + o["text"].toString());
          chat_status_->setText("Assistant: " + o["phase"].toString());
        }, Qt::QueuedConnection);
      });
  }

  void save(rviz_common::Config config) const override
  {
    Panel::save(config);
    config.mapSetValue("extra_args", args_->text());
  }

  void load(const rviz_common::Config & config) override
  {
    Panel::load(config);
    QString s;
    if (config.mapGetString("extra_args", &s)) {
      args_->setText(s);
    }
  }

protected:
  bool eventFilter(QObject * obj, QEvent * e) override
  {
    if (obj == cam_ && e->type() == QEvent::Resize && !img_.isNull()) {
      onImage(img_);
    }
    if (obj == wrist_ && e->type() == QEvent::Resize && !wrist_img_.isNull()) {
      onWrist(wrist_img_);
    }
    if (obj == cam_ && e->type() == QEvent::MouseButtonPress && !shown_.isEmpty()) {
      // label -> pixmap (centred) -> full camera pixels (the board corners)
      const QPointF off((cam_->width() - shown_.width()) / 2.0, (cam_->height() - shown_.height()) / 2.0);
      const QPointF p = (static_cast<QMouseEvent *>(e)->localPos() - off) * (full_w_ / shown_.width());
      for (const auto & [variant, poly] : polys_) {
        if (poly.containsPoint(p, Qt::OddEvenFill)) {
          if (variant == "UNKWN") {
            line("[panel] that board is UNKWN (variant unclear): not pickable");
          } else {
            start(variant, "image click");
          }
          return true;
        }
      }
    }
    return Panel::eventFilter(obj, e);
  }

private:
  void chatControl(const char * command)
  {
    if (chat_control_pub_) {std_msgs::msg::String m; m.data = command; chat_control_pub_->publish(m);}
  }

  void publishState()
  {
    if (!state_pub_) {return;}
    const QJsonObject o{{"stamp", QDateTime::currentMSecsSinceEpoch() / 1000.0},
      {"hardware", hw_kind_}, {"busy", running()}, {"waiting", waiting_},
      {"task_id", task_id_}, {"phase", waiting_ ? QString("waiting") : phase_}, {"detail", detail_}};
    std_msgs::msg::String m; m.data = QJsonDocument(o).toJson(QJsonDocument::Compact).toStdString();
    state_pub_->publish(m);
  }

  static QImage toQImage(const sensor_msgs::msg::Image & m)
  {
    if (m.encoding != "bgr8" && m.encoding != "rgb8") {return {};}
    QImage img(m.data.data(), m.width, m.height, m.step, QImage::Format_RGB888);
    return m.encoding == "bgr8" ? img.rgbSwapped() : img.copy();  // copy: m.data dies with the message
  }

  void onWrist(const QImage & img)
  {
    wrist_img_ = img;
    wrist_ms_ = QDateTime::currentMSecsSinceEpoch();
    wrist_->show();
    wrist_->setPixmap(QPixmap::fromImage(img).scaled(wrist_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }

  void onImage(const QImage & img)
  {
    img_ = img;
    const QPixmap pm = QPixmap::fromImage(img).scaled(cam_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    shown_ = pm.size();
    cam_->setPixmap(pm);
  }

  bool running() const {return proc_ && proc_->state() != QProcess::NotRunning;}

  void onBoards(const QString & json)
  {
    const auto o = QJsonDocument::fromJson(json.toUtf8()).object();
    free_.clear();
    under_.clear();
    polys_.clear();
    dests_.clear();
    unknown_ = 0;
    auto poly = [this](const QJsonObject & b) {
        QPolygonF q;
        for (const auto & c : b["corners"].toArray()) {
          q << QPointF(c.toArray()[0].toDouble(), c.toArray()[1].toDouble());
        }
        polys_.push_back({b["variant"].toString(), q});
      };
    for (const auto & b : o["boards"].toArray()) {
      if (b.toObject()["zone"].toString() != "pick") {continue;}  // drop-off table: shown, never picked
      const auto v = b.toObject()["variant"].toString();
      v == "UNKWN" ? ++unknown_ : ++free_[v];
      poly(b.toObject());
    }
    for (const auto & b : o["occluded"].toArray()) {
      if (b.toObject()["zone"].toString() != "pick") {continue;}
      ++under_[b.toObject()["variant"].toString()];
      poly(b.toObject());
    }
    full_w_ = o["image_size"].toArray()[0].toDouble(1920);
    for (const auto & d : o["destinations"].toArray()) {
      dests_.insert(d.toObject()["variant"].toString());
    }
    calibrated_ = o["calibrated"].toBool();
    source_ = o["source"].toString();
    live_ = o["live"].toBool(false);
    observed_ms_ = static_cast<qint64>(o["stamp"].toDouble() * 1000);
    last_ms_ = QDateTime::currentMSecsSinceEpoch();
    refresh();
  }

  // why variant v cannot be picked now, "" = it can
  QString blocked(const QString & v) const
  {
    const double age = (QDateTime::currentMSecsSinceEpoch() - last_ms_) / 1000.0;
    if (!last_ms_ || age > STALE_S) {return "no vision data";}
    const auto observed_age = QDateTime::currentMSecsSinceEpoch() - observed_ms_;
    if (!observed_ms_ || observed_age < 0 || observed_age > 2000) {return "stale observation timestamp";}
    if (hw_kind_ != "SIMULATION" && !live_) {return "replay cannot command real hardware";}
    if (!calibrated_) {return "camera not calibrated";}
    if (free_.value(v) + under_.value(v) == 0) {return "none detected";}
    if (!dests_.contains(v)) {return "its template is not visible";}
    if (running()) {return "a pick is running";}
    return "";
  }

  // green / red dot + text, by data age
  static QString dot(bool ok, const QString & text)
  {
    // ponytail: coloured block of spaces, not a dot glyph (the env's fontconfig crashes on font fallback for it)
    return QString("<span style='background:%1'>&nbsp;&nbsp;&nbsp;</span> %2").arg(ok ? "#43a047" : "#e53935", text);
  }

  void refresh()
  {
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const bool robot_ok = joints_ms_ && now - joints_ms_ < 1000;
    status_->setText(
      dot(robot_ok, robot_ok ? "robot: " + (hw_kind_.isEmpty() ? QString("connected") : hw_kind_) :
        QString("robot: not running")) + " &nbsp; " +
      dot(last_ms_ && now - last_ms_ < 2000, "overhead camera") + " &nbsp; " +
      dot(wrist_ms_ && now - wrist_ms_ < 2000, "wrist camera") + " &nbsp; " +
      dot(gripper_ == "IDLE" || gripper_ == "HOLDING", "gripper: " + (gripper_.isEmpty() ? QString("?") : gripper_)));
    const double age = (QDateTime::currentMSecsSinceEpoch() - last_ms_) / 1000.0;
    if (!last_ms_) {
      vision_->setText("vision: waiting for /vision/boards (pixi run vision)");
    } else if (age > STALE_S) {
      vision_->setText(QString("<b style='color:#e53935'>vision: no data for %1 s</b>").arg(age, 0, 'f', 0));
    } else {
      auto d = dests_.values();
      d.sort();
      vision_->setText(QString("vision: %1 | %2 | templates seen: %3%4")
        .arg(source_, calibrated_ ? "calibrated" : "<b>NOT calibrated</b>", d.isEmpty() ? "none" : d.join(" "),
        unknown_ ? QString(" | %1 UNKWN (not pickable)").arg(unknown_) : ""));
    }
    for (const auto & v : VARIANTS) {
      const QString why = blocked(v);
      const int u = under_.value(v);
      auto * b = buttons_[v];
      b->setText(QString("%1\n%2 free%3\n%4").arg(v).arg(free_.value(v))
        .arg(u ? QString(", %1 under").arg(u) : "").arg(dests_.contains(v) ? "template seen" : "no template"));
      b->setEnabled(why.isEmpty());
      b->setToolTip(why.isEmpty() ? "automatically pick, place, and release this variant" : why);
      b->setStyleSheet(why.isEmpty() ? "QPushButton{background:#2e7d32;color:white;font-weight:bold}" : "");
    }
    if (wrist_->isVisible() && QDateTime::currentMSecsSinceEpoch() - wrist_ms_ > 3000) {
      wrist_->hide();  // wrist camera stopped
    }
    stop_->setEnabled(running());
    confirm_->setEnabled(waiting_);
    confirm_->setStyleSheet(waiting_ ? "QPushButton{background:#f9a825;color:black;font-weight:bold}" : "");
    publishState();
  }

  void onRequest(const QString & req)
  {
    if (VARIANTS.contains(req)) {
      start(req, "topic");
    } else if (req == "confirm") {
      confirm("topic");
    } else if (req == "stop") {
      stop();
    } else {
      line("[panel] unknown request '" + req + "' (VARa / VARb / VARc / confirm / stop)");
    }
  }

  void start(const QString & v, const QString & src)
  {
    const QString why = blocked(v);
    if (!why.isEmpty()) {
      line(QString("[panel] cannot pick %1 (%2): %3").arg(v, src, why));
      return;
    }
    if (proc_) {
      proc_->deleteLater();
    }
    proc_ = new QProcess(this);
    proc_->setProcessChannelMode(QProcess::MergedChannels);
    connect(proc_, &QProcess::readyRead, this, &PickPanel::onOutput);
    connect(proc_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &PickPanel::onFinished);
    connect(proc_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError e) {
      if (e == QProcess::FailedToStart) {phase_ = "failed"; line("[panel] could not start python3: is RViz in the robot env?"); publishState();}
    });
    QStringList a = {"-u", PICK_ONE, "--place", "--variant", v};
    a += QProcess::splitCommand(args_->text());
    hw_.clear();
    buf_.clear();
    waiting_ = false;
    variant_ = v;
    line(QString("[panel] %1 requested (%2): python3 %3").arg(v, src, a.join(' ')));
    run_->setText(QString("<b>robot: pick and place %1 running</b>").arg(v));
    if (src != "conversation") {task_id_.clear();}
    phase_ = "running";
    detail_.clear();
    proc_->start("python3", a);
    refresh();
  }

  void confirm(const QString & src)
  {
    if (!waiting_) {
      line("[panel] nothing to confirm");
      return;
    }
    if (src != "button" && !hw_.startsWith("MOCK")) {
      line("[panel] confirm from " + src + " refused: robot is " + (hw_.isEmpty() ? "unknown" : hw_) +
        ". Press Confirm in the panel.");
      return;
    }
    proc_->write("\n");
    waiting_ = false;
    line("[panel] confirmed (" + src + ")");
    run_->setText(QString("<b>robot: pick and place %1 running</b>").arg(variant_));
    refresh();
  }

  void stop()
  {
    if (!running()) {
      return;
    }
    ::kill(proc_->processId(), SIGINT);
    phase_ = "stopping";
    line("[panel] Ctrl+C sent to pick_one");
    publishState();
    QTimer::singleShot(3000, this, [p = QPointer<QProcess>(proc_)] {
      if (p && p->state() != QProcess::NotRunning) {p->kill();}
    });
  }

  void onOutput()
  {
    buf_ += QString::fromUtf8(proc_->readAll());
    int i;
    while ((i = buf_.indexOf('\n')) >= 0) {
      line(buf_.left(i));
      buf_.remove(0, i + 1);
    }
    if (buf_.startsWith(">>>")) {  // input() prompt: no newline, python -u writes it in one piece
      line(buf_);
      run_->setText("<b style='color:#f9a825'>WAITING FOR YOU:</b> " + buf_.mid(4).toHtmlEscaped());
      buf_.clear();
      waiting_ = true;
      refresh();
    }
  }

  void onFinished(int code, QProcess::ExitStatus status)
  {
    if (!buf_.isEmpty()) {
      line(buf_);
      buf_.clear();
    }
    waiting_ = false;
    const bool ok = status == QProcess::NormalExit && code == 0;
    phase_ = ok ? "completed" : "failed";
    const bool stopped = status == QProcess::CrashExit || code == 130;  // Ctrl+C: python re-raises SIGINT
    if (stopped) {phase_ = "stopped";}
    line(QString("[panel] pick_one finished, %1").arg(stopped ? "stopped" : QString("exit code %1").arg(code)));
    const QString done = args_->text().contains("--hover-only") ? "hover done, check the cup position" : "placed";
    run_->setText(ok ? QString("<b style='color:#43a047'>robot: %1 %2</b>").arg(variant_, done) :
      stopped ? QString("<b style='color:#e53935'>robot: %1 stopped.</b> If the cup still holds the board, "
      "release it: pixi run shell, then: release").arg(variant_) :
      QString("<b style='color:#e53935'>robot: %1 failed (exit %2), see log</b>").arg(variant_).arg(code));
    refresh();
  }

  void line(const QString & s)
  {
    if (s.startsWith("robot hardware: ")) {
      hw_ = s.mid(16);
    }
    log_->appendPlainText(s);
    if (log_pub_) {
      std_msgs::msg::String m;
      m.data = s.toStdString();
      log_pub_->publish(m);
    }
  }

  QPlainTextEdit * chat_;
  QLineEdit * chat_input_;
  QLabel * chat_status_, * batch_status_;
  QString task_id_, phase_ = "idle", detail_;
  bool live_ = false;
  qint64 observed_ms_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, chat_pub_, chat_control_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chat_sub_, batch_sub_;
  QLabel * cam_, * wrist_, * vision_, * run_;
  QImage img_, wrist_img_;
  qint64 wrist_ms_ = 0, joints_ms_ = 0;
  QLabel * status_;
  QString hw_kind_, gripper_;
  QSize shown_;  // size of the scaled camera pixmap in cam_
  double full_w_ = 1920;  // camera width in px (board corners)
  std::vector<std::pair<QString, QPolygonF>> polys_;
  QLineEdit * args_;
  QPushButton * confirm_, * stop_;
  QPlainTextEdit * log_;
  QMap<QString, QPushButton *> buttons_;
  QMap<QString, int> free_, under_;
  QSet<QString> dests_;
  int unknown_ = 0;
  bool calibrated_ = false, waiting_ = false;
  QString source_, hw_, buf_, variant_;
  qint64 last_ms_ = 0;
  QProcess * proc_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr boards_sub_, request_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_, wrist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desc_sub_;
  rclcpp::Subscription<hk_fr5_interfaces::msg::GripperState>::SharedPtr gripper_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
};
}  // namespace vision1_rviz

PLUGINLIB_EXPORT_CLASS(vision1_rviz::PickPanel, rviz_common::Panel)
#include "pick_panel.moc"
