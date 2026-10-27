"""Isolated ROS/GPU saved-image test; does not connect to a robot or camera."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--settings',type=Path,default=Path('config/yolo_live.json'))
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--samples',type=int,default=5)
    p.add_argument('--min-boards',type=int,default=1)
    a=p.parse_args()
    if a.samples<1 or a.min_boards<0:p.error('samples must be positive; min-boards must be nonnegative')
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
    packets=deque(maxlen=100);debug=deque(maxlen=20)
    sub=node.create_subscription(String,'/perception/boards_pixels',lambda m:packets.append(json.loads(m.data)),1)
    visual=node.create_subscription(Image,'/perception/boards_pixels/debug_image',lambda m:debug.append(m),1)
    client=node.create_client(Trigger,'/perception/get_boards_pixels')
    def publish():
        msg=bridge.cv2_to_imgmsg(image,'bgr8');msg.header.stamp=node.get_clock().now().to_msg()
        msg.header.frame_id='scene_camera_optical_frame';pub.publish(msg)
    timer=node.create_timer(.2,publish)
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
                 '--work-dir',str(run/'worker'),'--period','.1']
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print('Isolated saved-image test; log:',run/'server.log',flush=True)
        assert until(lambda:any(v.get('status')=='ok' for v in packets) and bool(debug),150),'No fresh observation/debug image'
        observations=[]
        for _ in range(a.samples):
            requested=node.get_clock().now().nanoseconds*1e-9
            future=client.call_async(Trigger.Request());assert until(future.done,12),'Fresh service timed out'
            response=future.result();assert response.success,response.message
            packet=json.loads(response.message)
            assert packet['source_timestamp_sec']>=requested
            assert packet['model_backend']==expected_backend
            result=handoff(packet,node.get_clock().now().nanoseconds*1e-9)
            assert len(result['objects'])>=a.min_boards,'Replay needs positive board detections'
            assert all(o.get('class_name')!='hand' for o in result['objects'])
            observations.append(result)
        assert len({v['snapshot_id'] for v in observations})==a.samples,'Snapshot IDs reused'
        (run/'observations.json').write_text(json.dumps(observations,indent=2,allow_nan=False))
        stamp=packet['source_stamp']
        assert until(lambda:any(m.header.stamp.sec==stamp['sec'] and m.header.stamp.nanosec==stamp['nanosec'] for m in debug),3)
        assert all((m.height,m.width)==image.shape[:2] for m in debug)
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
        report={'passed':True,'boards':len(result['objects']),'hands':result['hand_count'],
                'detector_ms':packet['detector_ms'],'fresh_service':True,'matching_debug_stamp':True,
                'backend':expected_backend,'samples':a.samples,'unique_snapshots':True,
                'camera_loss_rejected':True,'camera_reconnect_recovered':True,'worker_crash_rejected':True,
                'ROS_DOMAIN_ID':173,'real_hardware_used':False,
                'model_accuracy_validated':False}
        (run/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
    finally:
        timer.cancel()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
