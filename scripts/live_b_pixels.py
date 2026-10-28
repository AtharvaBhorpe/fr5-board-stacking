"""Continuous latest-frame B observations and fresh-snapshot ROS service. No motion."""
import argparse
import contextlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from b_worker_protocol import read_reply
from read_live_b_pixels import handoff


def fresh(packet,now,max_age):
    return (packet is not None and packet.get('status')=='ok'
            and -.1<=now-packet['source_timestamp_sec']<=max_age)


def write_frame(path,image):
    import numpy as np
    # Lossless transfer between the two Python environments, without PNG compression.
    with path.open('wb') as stream:np.save(stream,image,allow_pickle=False)


def read_frame(path):
    import numpy as np
    image=np.load(path,allow_pickle=False)
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:
        raise ValueError('Expected a uint8 BGR image')
    return image


def worker(args):
    with contextlib.redirect_stdout(sys.stderr):
        import cv2
        import tomllib
        from b_live_geometry import measure,draw
        settings=json.loads(args.settings.read_text())
        config=tomllib.loads(Path(settings['scene_config']).read_text())
        backend=settings.get('backend','grounded_sam2')
        if backend=='yolo11_seg':
            from yolo_pixels import YoloModels
            models=YoloModels(settings)
        elif backend=='grounded_sam2':
            from inspect_b import Models
            models=Models(settings)
        else:raise ValueError('Unknown perception backend: '+str(backend))
    print(json.dumps({'ready':True}),flush=True)
    for line in sys.stdin:
        try:
            worker_started=time.monotonic()
            request=json.loads(line)
            image=read_frame(request['image'])
            read_ms=(time.monotonic()-worker_started)*1000
            with contextlib.redirect_stdout(sys.stderr):
                if backend=='yolo11_seg':report,accepted,masks,roi=models.detect(image,config,include_masks=False)
                else:report,accepted,masks,roi=models.detect(image,config)
            objects=[measure(candidate,mask) for candidate,mask in accepted]
            uncertain=[measure(candidate) for candidate in report.get('uncertain_candidates',[])]
            hands=report.get('hand_detections',[])
            if request.get('overlay'):
                overlay=Path(request['overlay'])
                visual=draw(image,uncertain+objects)
                if hands:
                    from yolo_pixels import draw_hands
                    draw_hands(visual,hands)
                if not cv2.imwrite(str(overlay),visual):raise OSError('Overlay save failed')
            response={'ok':True,'objects':objects,'uncertain_objects':uncertain,'image_size':report['image_size'],
                      'geometry_image_size':report.get('geometry_image_size',report['image_size']),
                      'geometry_pixel_scale':report.get('geometry_pixel_scale',[1.,1.]),
                      'model_backend':report.get('model_backend','grounded_sam2'),
                      'checkpoint_sha256':report.get('checkpoint_sha256'),
                      'hand_detections':hands,
                      'detector_ms':report['detector_ms'],
                      'worker_ms':(time.monotonic()-worker_started)*1000,
                      'frame_transfer_read_ms':read_ms,
                      'prompt_refinement':report.get('prompt_refinement'),
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
    cv2.setNumThreads(1)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge

    run=args.work_dir/uuid.uuid4().hex
    run.mkdir(parents=True,exist_ok=False)
    command=worker_command(Path.cwd(),args.settings)
    process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    print('Loading perception model once on GPU; waiting for worker...',flush=True)
    node=None;executor=None;service_node=None;service_executor=None;service_thread=None;frame_buffer=None
    try:
        live_settings=json.loads(args.settings.read_text())
        buffer_directory=live_settings.get('frame_buffer_directory',str(run))
        preview_period=live_settings.get('preview_period_s',.2)
        if not math.isfinite(preview_period) or preview_period<=0:raise ValueError('Invalid preview_period_s')
        frame_buffer=tempfile.TemporaryDirectory(prefix='fr5_frames_',dir=buffer_directory)
        if read_reply(process,120).get('ready') is not True:raise RuntimeError('Worker startup failed')
        rclpy.init()
        service_node=Node('live_b_pixels_snapshot')
        class LiveB(Node):
            def __init__(self):
                super().__init__('live_b_pixels')
                self.condition=threading.Condition();self.stop=threading.Event()
                self.latest=None;self.received=0;self.cached=None;self.bridge=CvBridge()
                group=ReentrantCallbackGroup()
                self.sub=self.create_subscription(Image,'/scene/image_raw',self.receive,
                    qos_profile_sensor_data,callback_group=group,raw=True)
                self.pub=self.create_publisher(String,'/perception/boards_pixels',1)
                self.debug=self.create_publisher(Image,'/perception/boards_pixels/debug_image',1)
                self.service=service_node.create_service(Trigger,'/perception/get_boards_pixels',self.snapshot)
                self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
                self.get_logger().info('READY: continuous /perception/boards_pixels and /perception/get_boards_pixels')

            def now_seconds(self):return self.get_clock().now().nanoseconds*1e-9

            def receive(self,msg):
                with self.condition:
                    self.received+=1;self.latest=(self.received,msg);self.condition.notify_all()

            def publish_error(self,error):
                packet={'status':'error','objects':[],'error':str(error),'execution_allowed':False}
                with self.condition:self.cached=None;self.condition.notify_all()
                self.pub.publish(String(data=json.dumps(packet)))
                self.get_logger().error(str(error))

            def loop(self):
                last=None;last_source_stamp=None;last_preview=0.
                while not self.stop.is_set():
                    with self.condition:
                        ready=self.condition.wait_for(lambda:self.stop.is_set() or (
                            self.latest is not None and self.latest[0]!=last),timeout=1)
                        if self.stop.is_set():return
                        if not ready:
                            if self.cached is not None and not fresh(self.cached,self.now_seconds(),args.max_age):
                                self.publish_error('Camera stopped producing fresh images')
                            continue
                        last,raw=self.latest
                    started=time.monotonic()
                    try:
                        # Deserialize only the newest frame, rather than every queued ROS image.
                        msg=deserialize_message(raw,Image)
                        source_stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
                        if source_stamp==last_source_stamp:
                            if self.cached is not None and not fresh(self.cached,self.now_seconds(),args.max_age):
                                self.publish_error('Camera source timestamp stopped advancing')
                            continue
                        last_source_stamp=source_stamp
                        stamp=source_stamp[0]+source_stamp[1]*1e-9
                        if not -.1<=self.now_seconds()-stamp<=args.max_age:
                            raise ValueError('Latest camera image is stale or clock-incompatible')
                        if msg.header.frame_id!='scene_camera_optical_frame':raise ValueError('Unexpected scene image frame')
                        image=self.bridge.imgmsg_to_cv2(msg,'bgr8')
                        decode_ms=(time.monotonic()-started)*1000
                        transfer_started=time.monotonic()
                        temporary=Path(frame_buffer.name)/'input_next.npy'
                        input_path=Path(frame_buffer.name)/'input.npy';overlay=run/'overlay.bmp'
                        write_frame(temporary,image)
                        temporary.replace(input_path)
                        transfer_ms=(time.monotonic()-transfer_started)*1000
                        preview=self.debug.get_subscription_count()>0 and started-last_preview>=preview_period
                        process.stdin.write(json.dumps({'image':str(input_path.resolve()),
                            'overlay':str(overlay.resolve()) if preview else None})+'\n')
                        process.stdin.flush()
                        reply_started=time.monotonic()
                        result=read_reply(process,30)
                        reply_ms=(time.monotonic()-reply_started)*1000
                        if not result.get('ok'):raise RuntimeError(result.get('error','Worker error'))
                        age=self.now_seconds()-stamp
                        snapshot=uuid.uuid4().hex
                        for index,item in enumerate(result['objects'],1):
                            item['instance_id']=f'{snapshot}:{index}'
                        for index,item in enumerate(result.get('uncertain_objects',[]),1):
                            item['instance_id']=f'{snapshot}:uncertain:{index}'
                        for index,item in enumerate(result.get('hand_detections',[]),1):
                            item['instance_id']=f'{snapshot}:hand:{index}'
                        packet={'status':'ok' if -.1<=age<=args.max_age else 'stale',
                                'coordinate_system':'raw_image_pixels','frame_id':msg.header.frame_id,
                                'source_timestamp_sec':stamp,'source_stamp':{'sec':source_stamp[0],'nanosec':source_stamp[1]},
                                'snapshot_id':snapshot,'observation_age_s':age,
                                'image_size':result['image_size'],'detector_ms':result['detector_ms'],
                                'geometry_image_size':result.get('geometry_image_size',result['image_size']),
                                'geometry_pixel_scale':result.get('geometry_pixel_scale',[1.,1.]),
                                'model_backend':result.get('model_backend','grounded_sam2'),
                                'checkpoint_sha256':result.get('checkpoint_sha256'),
                                'hand_detections':result.get('hand_detections',[]),
                                'human_safety_validated':False,
                                'prompt_refinement':result.get('prompt_refinement'),
                                'frame_transfer_write_ms':transfer_ms,
                                'worker_ms':result.get('worker_ms'),
                                'frame_transfer_read_ms':result.get('frame_transfer_read_ms'),
                                'ros_image_decode_ms':decode_ms,'worker_roundtrip_ms':reply_ms,
                                'received_frame_count':last,
                                'pipeline_ms':(time.monotonic()-started)*1000,
                                'objects':result['objects'],'rejected_proposals':result['rejected_proposals'],
                                'uncertain_objects':result.get('uncertain_objects',[]),
                                'execution_allowed':False,
                                'angle_convention':'clockwise image angle from +u/right, modulo 180 degrees',
                                'limits':['IDs are scoped to this snapshot, not persistent tracks.',
                                          'Pixel positions/angles require validated geometry before robot use.',
                                          'Supported mask quadrilaterals do not certify instance separation.']}
                        # Validate the same contract used by the motion-side reader before publishing.
                        handoff(packet,self.now_seconds(),args.max_age)
                        text=json.dumps(packet,allow_nan=False)
                        with self.condition:self.cached=packet;self.condition.notify_all()
                        self.pub.publish(String(data=text))
                        temp_report=run/'latest_next.json';temp_report.write_text(text);temp_report.replace(run/'latest.json')
                        visual=cv2.imread(str(overlay)) if preview else None
                        if visual is not None:
                            debug=self.bridge.cv2_to_imgmsg(visual,'bgr8');debug.header=msg.header
                            self.debug.publish(debug)
                            last_preview=started
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
        node=LiveB();executor=SingleThreadedExecutor();executor.add_node(node)
        # A blocking fresh-snapshot request must not stall camera subscriptions.
        service_executor=SingleThreadedExecutor();service_executor.add_node(service_node)
        def spin_service():
            with contextlib.suppress(ExternalShutdownException):service_executor.spin()
        service_thread=threading.Thread(target=spin_service,daemon=True);service_thread.start()
        try:executor.spin()
        except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:
            node.stop.set()
            with node.condition:node.condition.notify_all()
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        if node is not None:node.thread.join(timeout=3)
        for stream in (process.stdin,process.stdout):
            try:stream.close()
            except BrokenPipeError:pass
        if executor is not None:executor.shutdown()
        if service_executor is not None:service_executor.shutdown()
        if service_thread is not None:service_thread.join(timeout=3)
        if service_node is not None:service_node.destroy_node()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if frame_buffer is not None:frame_buffer.cleanup()


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
