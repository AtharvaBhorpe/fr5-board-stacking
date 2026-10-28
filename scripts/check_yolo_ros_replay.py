"""Isolated ROS/GPU saved-image test; does not connect to a robot or camera."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--settings',type=Path,default=Path('config/yolo_live.json'))
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--samples',type=int,default=5)
    p.add_argument('--min-boards',type=int,default=1)
    p.add_argument('--max-boards',type=int)
    p.add_argument('--input-fps',type=float,default=5.)
    p.add_argument('--min-update-fps',type=float,default=0.)
    p.add_argument('--stream-seconds',type=float,default=10.)
    a=p.parse_args()
    if a.samples<1 or a.min_boards<0:p.error('samples must be positive; min-boards must be nonnegative')
    if a.max_boards is not None and a.max_boards<a.min_boards:p.error('max-boards must be at least min-boards')
    if not math.isfinite(a.input_fps) or a.input_fps<=0:p.error('input-fps must be positive')
    if not math.isfinite(a.min_update_fps) or a.min_update_fps<0:p.error('min-update-fps must be nonnegative')
    if not math.isfinite(a.stream_seconds) or a.stream_seconds<=0:p.error('stream-seconds must be positive')
    expected_backend=json.loads(a.settings.read_text()).get('backend','grounded_sam2')
    root=Path.cwd();run=root/'.local'/f'yolo_ros_replay_{time.time_ns()}'
    run.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='173',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(run/'ros_logs'))
    os.environ.pop('ROS_DISCOVERY_SERVER',None)
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rclpy.qos import qos_profile_sensor_data
    from read_live_b_pixels import handoff
    image=cv2.imread(str(a.image))
    if image is None:raise ValueError('Cannot read saved image')
    rclpy.init();node=rclpy.create_node('yolo_isolated_replay_test');bridge=CvBridge()
    pub=node.create_publisher(Image,'/scene/image_raw',qos_profile_sensor_data)
    window_size=max(200,min(10000,math.ceil(a.input_fps*(a.stream_seconds+2))))
    packets=deque(maxlen=100);debug=deque(maxlen=20);receipts=deque(maxlen=window_size)
    def receive(message):
        packet=json.loads(message.data);packets.append(packet)
        if packet.get('status')=='ok':
            receipts.append((time.monotonic(),packet['snapshot_id'],packet['detector_ms'],packet))
    sub=node.create_subscription(String,'/perception/boards_pixels',receive,1)
    visual=node.create_subscription(Image,'/perception/boards_pixels/debug_image',lambda m:debug.append(m),1)
    client=node.create_client(Trigger,'/perception/get_boards_pixels')
    published=deque(maxlen=window_size);published_stamps=deque(maxlen=10000)
    msg=bridge.cv2_to_imgmsg(image,'bgr8');frozen=False
    def publish():
        if not frozen:msg.header.stamp=node.get_clock().now().to_msg()
        msg.header.frame_id='scene_camera_optical_frame';pub.publish(msg)
        published.append(time.monotonic())
        published_stamps.append((msg.header.stamp.sec,msg.header.stamp.nanosec))
    timer=node.create_timer(1/a.input_fps,publish)
    process=None;log=(run/'server.log').open('w')
    def until(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return True
            if process is not None and process.poll() is not None:
                raise RuntimeError('Worker exited; inspect '+str(run/'server.log'))
        return False
    try:
        command=[sys.executable,str(root/'scripts/live_b_pixels.py'),'serve','--settings',str(a.settings.resolve()),
                 '--work-dir',str(run/'worker'),'--period',str(min(.1,.5/a.input_fps))]
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print('Isolated saved-image test; log:',run/'server.log',flush=True)
        assert until(lambda:any(v.get('status')=='ok' for v in packets) and bool(debug),150),'No fresh observation/debug image'
        observations=[];receipts.clear();published.clear()
        # Measure streaming separately from sequential fresh-snapshot requests.
        streaming_started=time.monotonic()
        until(lambda:time.monotonic()-streaming_started>=a.stream_seconds,a.stream_seconds+2)
        received=list({item[1]:item for item in receipts}.values())
        assert len(received)>1,'No sustained stream of unique processed images'
        update_fps=(len(received)-1)/(received[-1][0]-received[0][0]) if len(received)>1 else None
        detector_median=statistics.median(item[2] for item in received) if received else None
        source_fps=(len(published)-1)/(published[-1]-published[0]) if len(published)>1 else None
        timing_fields=('pipeline_ms','worker_ms','frame_transfer_write_ms','frame_transfer_read_ms',
                       'ros_image_decode_ms','worker_roundtrip_ms')
        timing_medians={k:statistics.median(i[3][k] for i in received) for k in timing_fields}
        incoming_fps=(received[-1][3]['received_frame_count']-received[0][3]['received_frame_count'])/(received[-1][0]-received[0][0])
        ages=[item[3]['observation_age_s']*1000 for item in received]
        age_metrics={'median_ms':statistics.median(ages),'p95_ms':statistics.quantiles(ages,n=20)[18]}
        print(json.dumps({'received_updates_per_second':update_fps,'median_detector_ms':detector_median,
                         'published_images_per_second':source_fps,'received_images_per_second':incoming_fps,
                         'timing_medians_ms':timing_medians,'source_to_result_age':age_metrics}),flush=True)
        for _ in range(a.samples):
            requested=node.get_clock().now().nanoseconds*1e-9
            future=client.call_async(Trigger.Request());assert until(future.done,12),'Fresh service timed out'
            response=future.result();assert response.success,response.message
            packet=json.loads(response.message)
            assert packet['source_timestamp_sec']>=requested
            assert packet['model_backend']==expected_backend
            result=handoff(packet,node.get_clock().now().nanoseconds*1e-9)
            assert len(result['objects'])>=a.min_boards,'Replay needs positive board detections'
            if a.max_boards is not None:assert len(result['objects'])<=a.max_boards,'Unexpected board detections'
            assert all(o.get('class_name')!='hand' for o in result['objects'])
            observations.append(result)
        assert len({v['snapshot_id'] for v in observations})==a.samples,'Snapshot IDs reused'
        (run/'observations.json').write_text(json.dumps(observations,indent=2,allow_nan=False))
        assert all(m.header.frame_id=='scene_camera_optical_frame' and
                   (m.header.stamp.sec,m.header.stamp.nanosec) in published_stamps for m in debug)
        assert all((m.height,m.width)==image.shape[:2] for m in debug)
        frozen=True;packets.clear();frozen_at=time.monotonic()
        until(lambda:time.monotonic()-frozen_at>=1.,2)
        frozen_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        future=client.call_async(Trigger.Request());assert until(future.done,10)
        assert not future.result().success,'Repeated camera timestamp accepted as a fresh image'
        assert not any(v.get('status')=='ok' and v['source_timestamp_sec']>frozen_stamp for v in packets)
        assert sum(v.get('status')=='ok' and v['source_timestamp_sec']==frozen_stamp for v in packets)<=1,'Repeated source produced duplicate observations'
        frozen=False;packets.clear()
        assert until(lambda:any(v.get('status')=='ok' for v in packets),12),'Timestamp recovery failed'
        timer.cancel();packets.clear()
        assert until(lambda:any(v.get('status')=='error' for v in packets),8),'Camera stop failed to invalidate'
        future=client.call_async(Trigger.Request());assert until(future.done,10)
        assert not future.result().success,'Stale service response accepted'
        packets.clear();timer.reset()
        assert until(lambda:any(v.get('status')=='ok' for v in packets),12),'Camera reconnect did not recover'
        # Kill only descendants of this isolated test server, never another perception process.
        children=[];parents=[process.pid]
        while parents:
            parent=parents.pop()
            # Pixi can spawn the Python worker from a runtime thread.
            for child_file in Path(f'/proc/{parent}/task').glob('*/children'):
                try:found=[int(v) for v in child_file.read_text().split()]
                except FileNotFoundError:continue
                found=[pid for pid in found if pid not in children]
                children.extend(found);parents.extend(found)
        assert children,'GPU worker process missing'
        packets.clear()
        for pid in reversed(children):
            try:os.kill(pid,signal.SIGKILL)
            except ProcessLookupError:pass
        assert until(lambda:any(v.get('status')=='error' for v in packets),8),'Worker crash did not invalidate output'
        future=client.call_async(Trigger.Request());assert until(future.done,10)
        assert not future.result().success,'Worker failure served a cached board'
        cadence_passed=not a.min_update_fps or (update_fps is not None and update_fps>=a.min_update_fps)
        report={'passed':bool(cadence_passed),'transport_checks_passed':True,
                'requested_cadence_passed':bool(cadence_passed),'boards':len(result['objects']),'hands':result['hand_count'],
                'detector_ms':packet['detector_ms'],'fresh_service':True,'matching_debug_stamp':True,
                'median_detector_ms':detector_median,'received_updates_per_second':update_fps,
                'published_images_per_second':source_fps,
                'received_images_per_second':incoming_fps,'timing_medians_ms':timing_medians,
                'source_to_result_age':age_metrics,'stream_seconds':a.stream_seconds,
                'timing_unique_updates':len(received),'replay_input_fps':a.input_fps,
                'backend':expected_backend,'samples':a.samples,'unique_snapshots':True,
                'camera_loss_rejected':True,'camera_reconnect_recovered':True,'worker_crash_rejected':True,
                'repeated_source_timestamp_rejected':True,
                'ROS_DOMAIN_ID':173,'real_hardware_used':False,
                'model_accuracy_validated':False}
        (run/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
        if a.min_update_fps:
            assert update_fps is not None and update_fps>=a.min_update_fps,'Received update cadence below requested minimum'
    finally:
        timer.cancel()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
