// RViz panel: overhead camera with detections (/vision/image) and the board variants that can be picked now
// (/vision/boards). Click a variant button, or a board in the camera image = supervised pick and place.
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
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <sensor_msgs/msg/image.hpp>
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
    auto * lay = new QVBoxLayout(this);
    setMinimumWidth(560);
    cam_ = new QLabel("camera: waiting for /vision/image");
    cam_->setAlignment(Qt::AlignCenter);
    cam_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);  // no resize loop from the pixmap
    cam_->setMinimumSize(480, 270);
    cam_->setStyleSheet("background:black;color:white");
    cam_->setToolTip("click a board = pick and place it");
    cam_->installEventFilter(this);
    lay->addWidget(cam_, 3);
    vision_ = new QLabel("vision: waiting for /vision/boards (pixi run vision)");
    vision_->setWordWrap(true);
    lay->addWidget(vision_);
    lay->addWidget(new QLabel("<b>Click a variant (or a board in the image) to pick and place it:</b>"));
    auto * row = new QHBoxLayout;
    for (const auto & v : VARIANTS) {
      auto * b = new QPushButton(v);
      b->setMinimumHeight(105);
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
    stop_->setToolTip("Ctrl+C to pick_one. A motion already sent still finishes: the e-stop is on the pendant.");
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
    lay->addWidget(log_, 1);
    auto * t = new QTimer(this);
    connect(t, &QTimer::timeout, this, &PickPanel::refresh);
    t->start(500);
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
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/vision/image", 2, [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        if (m->encoding != "bgr8") {return;}
        QImage img = QImage(m->data.data(), m->width, m->height, m->step, QImage::Format_RGB888).rgbSwapped();
        QMetaObject::invokeMethod(this, [this, img] {onImage(img);}, Qt::QueuedConnection);
      });
    log_pub_ = node_->create_publisher<std_msgs::msg::String>("/pick_panel/log", 50);
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
      const auto v = b.toObject()["variant"].toString();
      v == "UNKWN" ? ++unknown_ : ++free_[v];
      poly(b.toObject());
    }
    for (const auto & b : o["occluded"].toArray()) {
      ++under_[b.toObject()["variant"].toString()];
      poly(b.toObject());
    }
    full_w_ = o["image_size"].toArray()[0].toDouble(1920);
    for (const auto & d : o["destinations"].toArray()) {
      dests_.insert(d.toObject()["variant"].toString());
    }
    calibrated_ = o["calibrated"].toBool();
    source_ = o["source"].toString();
    last_ms_ = QDateTime::currentMSecsSinceEpoch();
    refresh();
  }

  // why variant v cannot be picked now, "" = it can
  QString blocked(const QString & v) const
  {
    const double age = (QDateTime::currentMSecsSinceEpoch() - last_ms_) / 1000.0;
    if (!last_ms_ || age > STALE_S) {return "no vision data";}
    if (!calibrated_) {return "camera not calibrated";}
    if (free_.value(v) + under_.value(v) == 0) {return "none detected";}
    if (!dests_.contains(v)) {return "its template is not visible";}
    if (running()) {return "a pick is running";}
    return "";
  }

  void refresh()
  {
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
      b->setToolTip(why.isEmpty() ? "pick and place this variant (you confirm each descent)" : why);
      b->setStyleSheet(why.isEmpty() ? "QPushButton{background:#2e7d32;color:white;font-weight:bold}" : "");
    }
    stop_->setEnabled(running());
    confirm_->setEnabled(waiting_);
    confirm_->setStyleSheet(waiting_ ? "QPushButton{background:#f9a825;color:black;font-weight:bold}" : "");
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
      if (e == QProcess::FailedToStart) {line("[panel] could not start python3: is RViz in the robot env?");}
    });
    QStringList a = {"-u", PICK_ONE, "--place", "--variant", v};
    a += QProcess::splitCommand(args_->text());
    hw_.clear();
    buf_.clear();
    waiting_ = false;
    variant_ = v;
    line(QString("[panel] %1 requested (%2): python3 %3").arg(v, src, a.join(' ')));
    run_->setText(QString("<b>robot: pick and place %1 running</b>").arg(v));
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
    line("[panel] Ctrl+C sent to pick_one");
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
    const bool stopped = status == QProcess::CrashExit;  // Ctrl+C: python re-raises SIGINT
    line(QString("[panel] pick_one finished, %1").arg(stopped ? "stopped" : QString("exit code %1").arg(code)));
    run_->setText(ok ? QString("<b style='color:#43a047'>robot: %1 placed</b>").arg(variant_) :
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

  QLabel * cam_, * vision_, * run_;
  QImage img_;
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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
};
}  // namespace vision1_rviz

PLUGINLIB_EXPORT_CLASS(vision1_rviz::PickPanel, rviz_common::Panel)
#include "pick_panel.moc"
