"""Continuous latest-frame B observations and fresh-snapshot ROS service. No motion."""
import argparse
import contextlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import uuid
from b_worker_protocol import read_reply


def fresh(packet,now,max_age):
    return (packet is not None and packet.get('status')=='ok'
            and -.1<=now-packet['source_timestamp_sec']<=max_age)


def worker(args):
    with contextlib.redirect_stdout(sys.stderr):
        import cv2
        import tomllib
        from inspect_b import Models
        from b_live_geometry import measure,draw
        settings=json.loads(args.settings.read_text())
        config=tomllib.loads(Path(settings['scene_config']).read_text())
        models=Models(settings)
    print(json.dumps({'ready':True}),flush=True)
    for line in sys.stdin:
        try:
            request=json.loads(line)
            image=cv2.imread(request['image'])
            if image is None:raise ValueError('Unreadable worker image')
            with contextlib.redirect_stdout(sys.stderr):
                report,accepted,masks,roi=models.detect(image,config)
            objects=[measure(candidate,mask) for candidate,mask in accepted]
            overlay=Path(request['overlay'])
            if not cv2.imwrite(str(overlay),draw(image,objects)):raise OSError('Overlay save failed')
            response={'ok':True,'objects':objects,'image_size':report['image_size'],
                      'detector_ms':report['detector_ms'],
                      'rejected_proposals':report['rejected_proposals']}
        except Exception as error:
            response={'ok':False,'error':str(error)}
        print(json.dumps(response,allow_nan=False),flush=True)


def worker_command(root,settings):
    pixi=root/'.tools/pixi'
    if not pixi.is_file():
        executable=shutil.which('pixi')
        if executable is None:raise RuntimeError('Pixi is required to start the GPU environment')
        pixi=Path(executable)
    # ROS Python 3.14 paths must not leak into the GPU worker's Python 3.12.
    return [str(pixi),'run','--frozen','--quiet','--clean-env','-e','perception-gpu',
            'python',str(Path(__file__).resolve()),'worker','--settings',str(settings.resolve())]


def serve(args):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge

    run=args.work_dir/uuid.uuid4().hex
    run.mkdir(parents=True,exist_ok=False)
    command=worker_command(Path.cwd(),args.settings)
    process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    print('Loading B once on GPU; waiting for worker...',flush=True)
    node=None;executor=None
    try:
        if read_reply(process,120).get('ready') is not True:raise RuntimeError('Worker startup failed')
        rclpy.init()
        class LiveB(Node):
            def __init__(self):
                super().__init__('live_b_pixels')
                self.condition=threading.Condition();self.stop=threading.Event()
                self.latest=None;self.cached=None;self.bridge=CvBridge()
                group=ReentrantCallbackGroup()
                self.sub=self.create_subscription(Image,'/scene/image_raw',self.receive,
                    qos_profile_sensor_data,callback_group=group)
                self.pub=self.create_publisher(String,'/perception/boards_pixels',1)
                self.debug=self.create_publisher(Image,'/perception/boards_pixels/debug_image',1)
                self.service=self.create_service(Trigger,'/perception/get_boards_pixels',
                    self.snapshot,callback_group=group)
                self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
                self.get_logger().info('READY: continuous /perception/boards_pixels and /perception/get_boards_pixels')

            def now_seconds(self):return self.get_clock().now().nanoseconds*1e-9

            def receive(self,msg):
                with self.condition:
                    self.latest=msg;self.condition.notify_all()

            def publish_error(self,error):
                packet={'status':'error','objects':[],'error':str(error),'execution_allowed':False}
                with self.condition:self.cached=None;self.condition.notify_all()
                self.pub.publish(String(data=json.dumps(packet)))
                self.get_logger().error(str(error))

            def loop(self):
                last=None
                while not self.stop.is_set():
                    with self.condition:
                        ready=self.condition.wait_for(lambda:self.stop.is_set() or (
                            self.latest is not None and (self.latest.header.stamp.sec,self.latest.header.stamp.nanosec)!=last),timeout=1)
                        if self.stop.is_set():return
                        if not ready:
                            if self.cached is not None and not fresh(self.cached,self.now_seconds(),args.max_age):
                                self.publish_error('Camera stopped producing fresh images')
                            continue
                        msg=self.latest
                        last=(msg.header.stamp.sec,msg.header.stamp.nanosec)
                    started=time.monotonic()
                    try:
                        stamp=last[0]+last[1]*1e-9
                        if not -.1<=self.now_seconds()-stamp<=args.max_age:
                            raise ValueError('Latest camera image is stale or clock-incompatible')
                        if msg.header.frame_id!='scene_camera_optical_frame':raise ValueError('Unexpected scene image frame')
                        image=self.bridge.imgmsg_to_cv2(msg,'bgr8')
                        temporary=run/'input_next.png';input_path=run/'input.png';overlay=run/'overlay.jpg'
                        if not cv2.imwrite(str(temporary),image):raise OSError('Frame save failed')
                        temporary.replace(input_path)
                        process.stdin.write(json.dumps({'image':str(input_path.resolve()),'overlay':str(overlay.resolve())})+'\n')
                        process.stdin.flush()
                        result=read_reply(process,30)
                        if not result.get('ok'):raise RuntimeError(result.get('error','Worker error'))
                        age=self.now_seconds()-stamp
                        snapshot=uuid.uuid4().hex
                        for index,item in enumerate(result['objects'],1):
                            item['instance_id']=f'{snapshot}:{index}'
                        packet={'status':'ok' if -.1<=age<=args.max_age else 'stale',
                                'coordinate_system':'raw_image_pixels','frame_id':msg.header.frame_id,
                                'source_timestamp_sec':stamp,'source_stamp':{'sec':last[0],'nanosec':last[1]},
                                'snapshot_id':snapshot,'observation_age_s':age,
                                'image_size':result['image_size'],'detector_ms':result['detector_ms'],
                                'objects':result['objects'],'rejected_proposals':result['rejected_proposals'],
                                'execution_allowed':False,
                                'angle_convention':'clockwise image angle from +u/right, modulo 180 degrees',
                                'limits':['IDs are scoped to this snapshot, not persistent tracks.',
                                          'Pixel positions/angles require validated geometry before robot use.',
                                          'Supported mask quadrilaterals do not certify instance separation.']}
                        text=json.dumps(packet,allow_nan=False)
                        with self.condition:self.cached=packet;self.condition.notify_all()
                        self.pub.publish(String(data=text))
                        temp_report=run/'latest_next.json';temp_report.write_text(text);temp_report.replace(run/'latest.json')
                        visual=cv2.imread(str(overlay))
                        if visual is not None:
                            debug=self.bridge.cv2_to_imgmsg(visual,'bgr8');debug.header=msg.header
                            self.debug.publish(debug)
                    except Exception as error:
                        if not self.stop.is_set():self.publish_error(error)
                        if process.poll() is not None:return
                    self.stop.wait(max(0,args.period-(time.monotonic()-started)))

            def snapshot(self,request,response):
                after=self.now_seconds()
                with self.condition:
                    ready=self.condition.wait_for(lambda:self.stop.is_set() or (
                        fresh(self.cached,self.now_seconds(),args.max_age)
                        and self.cached['source_timestamp_sec']>=after),timeout=6)
                    if not ready or self.stop.is_set():
                        response.success=False;response.message='No fresh processed frame after request; check camera/worker';return response
                    packet=dict(self.cached)
                packet['observation_age_s']=self.now_seconds()-packet['source_timestamp_sec']
                response.success=True;response.message=json.dumps(packet,allow_nan=False)
                return response
        node=LiveB();executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
        try:executor.spin()
        except KeyboardInterrupt:pass
    finally:
        if node is not None:
            node.stop.set()
            with node.condition:node.condition.notify_all()
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        if node is not None:node.thread.join(timeout=3)
        if executor is not None:executor.shutdown()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def self_test():
    from b_live_geometry import self_test as geometry_test
    geometry_test()
    assert fresh({'status':'ok','source_timestamp_sec':10.},10.5,2.)
    assert not fresh({'status':'ok','source_timestamp_sec':10.},13.,2.)
    assert not fresh({'status':'error','source_timestamp_sec':10.},10.5,2.)
    assert not fresh(None,10.5,2.)
    assert not fresh({'status':'ok','source_timestamp_sec':11.},10.,2.)
    print('PASS: current/error/stale/future observation handling')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['serve','worker','self-test'])
    p.add_argument('--settings',type=Path,default=Path('config/pretrained_b_live.json'))
    p.add_argument('--work-dir',type=Path,default=Path('.local/live_b_pixels'))
    p.add_argument('--period',type=float,default=.5)
    p.add_argument('--max-age',type=float,default=2.)
    a=p.parse_args()
    if not all(math.isfinite(v) and v>0 for v in (a.period,a.max_age)):
        p.error('period and max-age must be finite and positive')
    if a.mode=='self-test':self_test()
    elif a.mode=='worker':worker(a)
    else:serve(a)
