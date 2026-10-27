"""Press S to save the displayed ROS scene frame and its dataset metadata. No motion."""
import argparse
from collections import deque
from datetime import datetime,timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import threading
import time
import uuid
import cv2
import numpy as np

CLASSES={'VARa':[295,210,4],'VARb':[210,152,4],'VARc':[210,142,4],'hand':None}


def stamp_seconds(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def nearest(messages,seconds,max_delta=.1):
    if not messages:return None
    value=min(messages,key=lambda m:abs(stamp_seconds(m)-seconds))
    return value if abs(stamp_seconds(value)-seconds)<=max_delta else None


def finite_json(value):
    if isinstance(value,(float,np.floating)):return float(value) if math.isfinite(value) else None
    if isinstance(value,dict):return {str(k):finite_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [finite_json(v) for v in value]
    if isinstance(value,np.ndarray):return finite_json(value.tolist())
    return value


def save_capture(session,image,metadata,attachments=None):
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:
        raise ValueError('Need original uint8 BGR camera pixels')
    age=metadata['saved_ros_time_sec']-metadata['source_timestamp_sec']
    if not math.isfinite(age) or not -.1<=age<=1.:
        raise ValueError('Displayed image is stale or its clock is incompatible; no capture saved')
    if shutil.disk_usage(session).free<64*1024*1024:raise OSError('Less than 64 MiB disk space available')
    capture_id='capture_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_'+uuid.uuid4().hex[:6]
    temporary=session/('.pending_'+capture_id);target=session/'samples'/capture_id
    temporary.mkdir();(session/'images').mkdir(exist_ok=True);target.parent.mkdir(exist_ok=True)
    try:
        scene_file=temporary/(capture_id+'.png')
        if not cv2.imwrite(str(scene_file),image):raise OSError('Scene PNG write failed')
        extra={}
        for name,array in (attachments or {}).items():
            if name not in ('wrist_rgb','wrist_depth'):raise ValueError('Unexpected attachment')
            if name=='wrist_rgb':
                filename=name+'.png'
                if not cv2.imwrite(str(temporary/filename),array):raise OSError('Wrist image write failed')
            else:
                filename=name+'.npy';np.save(temporary/filename,array,allow_pickle=False)
            extra[name]={'file':filename,'shape':list(array.shape),'dtype':str(array.dtype)}
        record=finite_json({**metadata,'capture_id':capture_id,'image_file':scene_file.name,
            'image_size':[image.shape[1],image.shape[0]],'encoding':'bgr8',
            'image_quality':{'laplacian_variance':float(cv2.Laplacian(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var()),
                             'mean_intensity':float(image.mean()),'quality_threshold_not_validated':True},
            'image_sha256':hashlib.sha256(scene_file.read_bytes()).hexdigest(),
            'attachments':extra,'classes_dimensions_mm':CLASSES,'annotations_pending':True,
            'execution_allowed':False})
        (temporary/'metadata.json').write_text(json.dumps(record,indent=2,allow_nan=False))
        temporary.rename(target)
        # Flat images/ is convenient for CVAT; the self-contained sample stays authoritative.
        try:os.link(target/scene_file.name,session/'images'/scene_file.name)
        except OSError:shutil.copyfile(target/scene_file.name,session/'images'/scene_file.name)
        return target,record
    except Exception:
        if temporary.exists():shutil.rmtree(temporary)
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--scene',type=int,default=13)
    p.add_argument('--guide',type=Path,default=Path('.local/capture_guide_100/scenes.json'))
    p.add_argument('--priority',action='store_true')
    p.add_argument('--notes',default='')
    p.add_argument('--with-wrist',action='store_true')
    p.add_argument('--with-robot',action='store_true')
    a=p.parse_args()
    if not 0<=a.scene<=100:p.error('scene must be between 0 and 100')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo,JointState
    from cv_bridge import CvBridge
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rcl_interfaces.srv import GetParameters
    session=(a.output_dir or Path('.local/datasets')/('webcam_capture_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])).resolve()
    session.mkdir(parents=True,exist_ok=False)
    guide=json.loads(a.guide.read_text()) if a.guide.is_file() else []
    scenes=[s for s in guide if not a.priority or s.get('priority')]
    ids=[s['id'] for s in scenes] or list(range(1,101))
    scene_id=a.scene;bridge=CvBridge();lock=threading.Lock();cache={'image':None,'parameters':None}
    histories={key:deque(maxlen=30 if 'info' in key or key=='joints' else 6)
               for key in ('scene_info','wrist_rgb','wrist_depth','wrist_info','depth_info','joints')}
    sources={'scene':'/scene/image_raw','scene_info':'/scene/camera_info',
        'wrist_rgb':'/wrist/wrist_camera/color/image_raw','wrist_depth':'/wrist/wrist_camera/aligned_depth_to_color/image_raw',
        'wrist_info':'/wrist/wrist_camera/color/camera_info','depth_info':'/wrist/wrist_camera/aligned_depth_to_color/camera_info',
        'joints':'/joint_states'}
    calibration=[]
    for relative in ['config/logitech_ros.yaml','config/perception_review_roi_v3.toml',
                     'config/calibration/logitech_scene.yaml','config/calibration/wrist_handeye_candidate_002.json',
                     'config/calibration/logitech_extrinsics_candidate_001.json']:
        path=Path(relative)
        if path.is_file():
            dest=session/'configuration_snapshot'/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,dest)
            calibration.append({'source':relative,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    session_meta={'created_utc':datetime.now(timezone.utc).isoformat(),'topics':sources,'classes_dimensions_mm':CLASSES,
        'operator_notes':a.notes,'configuration_files':calibration,'calibration_validated':False,
        'optional_wrist_requested':a.with_wrist,'optional_robot_requested':a.with_robot,
        'guide_poses_are_illustrative_not_measured':True}
    (session/'session.json').write_text(json.dumps(session_meta,indent=2));(session/'images').mkdir()
    rclpy.init();node=rclpy.create_node('capture_board_dataset');executor=SingleThreadedExecutor();executor.add_node(node)
    def receive(key,msg):
        with lock:
            if key=='image':cache['image']=msg
            else:histories[key].append(msg)
    subscriptions=[node.create_subscription(Image,sources['scene'],lambda m:receive('image',m),qos_profile_sensor_data),
        node.create_subscription(CameraInfo,sources['scene_info'],lambda m:receive('scene_info',m),qos_profile_sensor_data)]
    if a.with_wrist:
        for key,kind in [('wrist_rgb',Image),('wrist_depth',Image),('wrist_info',CameraInfo),('depth_info',CameraInfo)]:
            subscriptions.append(node.create_subscription(kind,sources[key],lambda m,k=key:receive(k,m),qos_profile_sensor_data))
    if a.with_robot:subscriptions.append(node.create_subscription(JointState,sources['joints'],lambda m:receive('joints',m),qos_profile_sensor_data))
    tf_buffer=None;tf_listener=None
    if a.with_robot:
        from tf2_ros import Buffer,TransformListener
        tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,node)
    params=node.create_client(GetParameters,'/scene/v4l2_camera/get_parameters')
    parameter_names=['focus_automatic_continuous','focus_absolute','auto_exposure','exposure_time_absolute','white_balance_automatic','white_balance_temperature']
    def parameters_done(future):
        try:
            response=future.result()
            with lock:cache['parameters']={'observed_ros_time_sec':node.get_clock().now().nanoseconds*1e-9,
                'values':{name:message_to_ordereddict(v) for name,v in zip(parameter_names,response.values)}}
        except Exception:pass
    pending=None
    def poll_parameters():
        nonlocal pending
        if params.service_is_ready() and (pending is None or pending.done()):
            request=GetParameters.Request();request.names=parameter_names
            pending=params.call_async(request);pending.add_done_callback(parameters_done)
    parameter_timer=node.create_timer(5.,poll_parameters);poll_parameters()
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    window='FR5 dataset capture | S save | N next | P previous | Q quit'
    cv2.namedWindow(window,cv2.WINDOW_NORMAL);cv2.resizeWindow(window,1600,850)
    cv2.createTrackbar('Guide scene',window,scene_id,100,lambda value:None)
    displayed=None;image=None;last=None;last_saved=None;guide_cache={};saved=0;status='Waiting for /scene/image_raw';colour=(0,200,255)
    print('CAPTURE SESSION:',session,flush=True)
    print('S saves the RAW displayed frame. N/P choose guide scene; trackbar jumps directly. Q exits.',flush=True)
    try:
        while rclpy.ok():
            scene_id=cv2.getTrackbarPos('Guide scene',window)
            with lock:msg=cache['image']
            if msg is not None:
                key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
                if key!=last:
                    try:image=bridge.imgmsg_to_cv2(msg,'bgr8').copy();displayed=msg;last=key
                    except Exception as error:status=str(error)
            canvas=np.full((850,1920,3),24,np.uint8)
            if image is not None:
                canvas[90:810,:1280]=cv2.resize(image,(1280,720))
                age=node.get_clock().now().nanoseconds*1e-9-stamp_seconds(displayed)
                cv2.putText(canvas,f'LIVE image age {age:.2f}s | saved {saved}',(15,37),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0) if -.1<=age<=1 else (0,0,255),2)
            target=next((s for s in guide if s['id']==scene_id),None)
            if target:
                path=a.guide.parent/'images'/f'scene_{scene_id:03}.png'
                if scene_id not in guide_cache:guide_cache[scene_id]=cv2.imread(str(path))
                visual=guide_cache[scene_id]
                if visual is not None:canvas[100:567,1280:1920]=cv2.resize(visual,(640,467))
            cv2.putText(canvas,f'Guide {scene_id:03} | S save | N/P scene | Q quit',(15,75),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
            cv2.putText(canvas,status[:135],(15,835),cv2.FONT_HERSHEY_SIMPLEX,.6,colour,2)
            cv2.imshow(window,canvas);key=cv2.waitKey(20)&255
            if key in (ord('q'),ord('Q'),27) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:break
            if key in (ord('n'),ord('N'),ord('p'),ord('P')):
                index=ids.index(scene_id) if scene_id in ids else 0
                scene_id=ids[(index+(1 if key in (ord('n'),ord('N')) else -1))%len(ids)]
                cv2.setTrackbarPos('Guide scene',window,scene_id)
            if key not in (ord('s'),ord('S')):continue
            if displayed is None or image is None:status='No scene image received - start the Logitech ROS publisher';continue
            if last==last_saved:status='Already saved this exact source frame; wait for the next live frame';continue
            try:
                seconds=stamp_seconds(displayed)
                with lock:
                    selected={k:nearest(v,seconds,.1) for k,v in histories.items()}
                    parameters=cache['parameters']
                info=selected['scene_info'];warnings=[]
                if info is None:warnings.append('No CameraInfo within 100 ms of this image')
                elif (info.width,info.height)!=(image.shape[1],image.shape[0]):warnings.append('CameraInfo resolution mismatch');info=None
                meta={'saved_utc':datetime.now(timezone.utc).isoformat(),'saved_ros_time_sec':node.get_clock().now().nanoseconds*1e-9,
                    'source_timestamp_sec':seconds,'source_header':message_to_ordereddict(displayed.header),'source_topic':sources['scene'],
                    'camera_info':message_to_ordereddict(info) if info is not None else None,
                    'observed_camera_parameters':parameters,'operator_notes':a.notes,'intended_guide_scene':target,
                    'suggested_arrangement_group':f'guide_{scene_id:03}','group_assignment_requires_review':True,
                    'calibration_files':calibration,'calibration_validated':False,'warnings':warnings,
                    'optional_streams':{},'joint_state':None,'robot_transforms':{},
                    'robot_feedback_verified_as_real_hardware':False}
                if parameters is None:warnings.append('Camera parameters unavailable; configuration snapshot is not proof of active settings')
                attachments={}
                for field in ('wrist_rgb','wrist_depth'):
                    sample=selected[field]
                    if a.with_wrist and sample is not None:
                        encoding='bgr8' if field=='wrist_rgb' else 'passthrough'
                        attachments[field]=bridge.imgmsg_to_cv2(sample,encoding).copy()
                        camera=selected['wrist_info' if field=='wrist_rgb' else 'depth_info']
                        meta['optional_streams'][field]={'header':message_to_ordereddict(sample.header),
                            'source_encoding':sample.encoding,'timestamp_delta_s':stamp_seconds(sample)-seconds,
                            'camera_info':message_to_ordereddict(camera) if camera is not None else None,
                            'raw_depth_unit_scale_verified':False if field=='wrist_depth' else None}
                    elif a.with_wrist:warnings.append(field+' missing or outside 100 ms window; not fabricated')
                if a.with_robot:
                    joints=selected['joints']
                    if joints is not None:meta['joint_state']=message_to_ordereddict(joints)
                    else:warnings.append('Measured joints missing or outside 100 ms window')
                    from rclpy.time import Time
                    for frame in ('flange','tcp'):
                        try:meta['robot_transforms'][frame]=message_to_ordereddict(tf_buffer.lookup_transform('base_link',frame,Time.from_msg(displayed.header.stamp)))
                        except Exception as error:meta['robot_transforms'][frame]={'available':False,'reason':str(error)}
                folder,record=save_capture(session,image,meta,attachments);saved+=1
                last_saved=last
                status=f'SAVED {saved}: {folder.name}'+(' | metadata warnings: '+str(len(warnings)) if warnings else '')
                colour=(0,255,0);print(json.dumps({'saved':str(folder),'image':str(session/'images'/record['image_file']),'guide_scene':scene_id,'warnings':warnings}),flush=True)
            except Exception as error:status='NOT SAVED: '+str(error);colour=(0,0,255);print(status,flush=True)
    finally:
        executor.shutdown();thread.join(timeout=3);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        cv2.destroyAllWindows();print('Saved captures:',saved,'| Images for CVAT:',session/'images',flush=True)


if __name__=='__main__':main()
