"""Measure B instances on a known parallel plane; ROS camera-frame preview only."""
import argparse
from collections import OrderedDict
import copy
import hashlib
import json
from pathlib import Path
import threading
import cv2
import numpy as np
from sorting_geometry import colour, classify, rays_to_plane, surface_pose


def check_camera(calibration, image_size, k, d):
    if image_size!=calibration['image_size']:
        raise ValueError('Image resolution differs from the metric reference')
    if (not np.allclose(np.asarray(k).reshape(3,3),calibration['camera_matrix'],atol=1e-5,rtol=0)
            or np.asarray(d).shape!=np.asarray(calibration['distortion_coefficients']).shape
            or not np.allclose(d,calibration['distortion_coefficients'],atol=1e-7,rtol=0)):
        raise ValueError('CameraInfo differs from the metric reference calibration')


def board_plane(calibration,settings,label,pixels,max_rms):
    if label not in settings['classes']:raise ValueError('Choose a configured reference class')
    length,width=settings['classes'][label]['size_m'][:2]
    objects=np.array([[-length/2,-width/2,0],[length/2,-width/2,0],
                      [length/2,width/2,0],[-length/2,width/2,0]],float)
    pixels=np.asarray(pixels,float)
    if pixels.shape!=(4,2) or not np.isfinite(pixels).all():raise ValueError('Need four reference corners')
    k=np.asarray(calibration['camera_matrix'],float);d=np.asarray(calibration['distortion_coefficients'],float)
    _,rotations,translations,_=cv2.solvePnPGeneric(objects,pixels,k,d,flags=cv2.SOLVEPNP_IPPE)
    options=[]
    for rv,t in zip(rotations,translations):
        rotation=cv2.Rodrigues(rv)[0]
        if np.min(objects@rotation[2]+t[2,0])<=0:continue
        projected=cv2.projectPoints(objects,rv,t,k,d)[0].reshape(4,2)
        rms=float(np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1))))
        options.append((rms,rotation,t.ravel()))
    if not options:raise ValueError('No valid rectangle pose; check the corner order')
    options.sort(key=lambda option:option[0]);rms,rotation,point=options[0]
    if rms>max_rms:raise ValueError(f'Reference fit RMS {rms:.2f} px exceeds {max_rms}; check corners/class')
    normal=rotation[:,2]
    if normal@(-point)<0:normal=-normal
    keys=('image_size','camera_matrix','distortion_coefficients','distortion_model')
    return {'camera_frame':'scene_camera_optical_frame',
            'calibration':{k:calibration[k] for k in keys},
            'reference_kind':'known_board_rectangle','reference_class':label,
            'reference_size_m':[length,width],'reference_corners_px':pixels.tolist(),
            'reference_reprojection_rms_px':rms,'pose_options_rms_px':[o[0] for o in options],
            'point_camera_m':point.tolist(),'normal_camera':normal.tolist(),
            'board_top_above_reference_m':0.,'physical_size_accuracy_validated':False,
            'scope':'Fixed camera; single-layer board tops coplanar with reference board top',
            'execution_allowed':False}


def click_reference(image):
    scale=min(1.,1280/image.shape[1],800/image.shape[0]);clicks=[]
    window='Known board: first LONG edge, then remaining corners in perimeter order'
    cv2.namedWindow(window)
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(clicks)<4:
            clicks.append([(x+.5)/scale-.5,(y+.5)/scale-.5])
    cv2.setMouseCallback(window,click)
    try:
        while True:
            view=cv2.resize(image,None,fx=scale,fy=scale)
            for i,q in enumerate(clicks):
                xy=tuple(np.rint(np.asarray(q)*scale).astype(int))
                cv2.circle(view,xy,5,(0,255,0),2)
                cv2.putText(view,str(i+1),xy,cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2)
            cv2.putText(view,'1-2 = LONG edge; 3-4 continue around SAME board; U undo; Enter save',
                        (10,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
            cv2.imshow(window,view);key=cv2.waitKey(30)&255
            if key==27:raise ValueError('Cancelled; plane was not saved')
            if key in (ord('u'),ord('U')) and clicks:clicks.pop()
            if key in (10,13) and len(clicks)==4:return clicks
    finally:cv2.destroyAllWindows()


def measure_packet(packet,image,settings,plane):
    if packet.get('status') not in ('ok','offline'):
        raise ValueError('B observation is stale or invalid')
    if packet.get('frame_id')!=plane['camera_frame']:
        raise ValueError('Wrong camera frame; never use Logitech calibration for wrist images')
    cal=plane['calibration']
    if (image is None or [image.shape[1],image.shape[0]]!=cal['image_size']
            or packet['image_size']!=cal['image_size']):
        raise ValueError('Source image missing or wrong resolution')
    normal=np.asarray(plane['normal_camera'],float)
    normal=normal/np.linalg.norm(normal)
    point=np.asarray(plane['point_camera_m'],float)+plane['board_top_above_reference_m']*normal
    identity={**cal,'T_base_camera':np.eye(4).tolist()}
    objects=[]
    for obj in packet['objects']:
        item={'instance_id':obj['instance_id'],'class_id':'unknown',
              'measurement_available':False,'reasons':[],
              'supported_pixel_corners':obj.get('supported_pixel_corners'),
              'robot_pose_available':False,'execution_allowed':False}
        try:
            corners=obj.get('supported_pixel_corners')
            if corners is None:raise ValueError('Mask has no supported complete four-corner outline')
            pixels=np.asarray(corners,float)
            if pixels.shape!=(4,2) or not np.isfinite(pixels).all():
                raise ValueError('Invalid board quadrilateral')
            xyz=rays_to_plane(pixels,identity,point,normal)
            pose,dimensions=surface_pose(xyz,normal)
            mask=np.zeros(image.shape[:2],np.uint8)
            cv2.fillConvexPoly(mask,np.rint(pixels).astype(np.int32),255)
            shade,fractions=colour(image,mask,settings)
            item.update(measurement_available=True,
                        measured_size_m=dimensions,
                        measured_size_mm=[v*1000 for v in dimensions],
                        centre_camera_m=pose[:3,3].tolist(),
                        long_axis_camera=pose[:3,0].tolist(),
                        T_camera_board=pose.tolist(),
                        visible_colour=shade,colour_measurements=fractions)
            try:
                label=classify(dimensions,shade,settings)
                item.update(class_id=label,destination_tag_id=settings['classes'][label]['destination_tag_id'],
                            expected_size_m=settings['classes'][label]['size_m'][:2])
            except ValueError as error:item['reasons'].append(str(error))
        except (ValueError,KeyError,cv2.error) as error:item['reasons'].append(str(error))
        objects.append(item)
    return {'status':packet['status'],'snapshot_id':packet['snapshot_id'],
            'source_timestamp_sec':packet.get('source_timestamp_sec'),
            'frame_id':plane['camera_frame'],'length_unit':'m','objects':objects,
            'reference_class':plane['reference_class'],
            'reference_kind':plane['reference_kind'],
            'physical_size_accuracy_validated':False,'robot_pose_available':False,
            'execution_allowed':False,
            'limits':['Reference scale/tilt and predicted outlines require independent ruler checks.',
                      'Known parallel board-top plane; unknown stacks/occlusion are outside this scope.',
                      'Camera-frame estimates are not base_link coordinates or motion approval.']}


def overlay(image,result):
    view=image.copy()
    for obj in result['objects']:
        pixels=obj.get('supported_pixel_corners')
        if pixels is None:continue
        q=np.rint(pixels).astype(np.int32)
        good=obj['measurement_available'] and obj['class_id']!='unknown'
        colour_value=(0,255,0) if good else (0,165,255)
        cv2.polylines(view,[q],True,colour_value,2)
        label=obj['class_id']
        if obj['measurement_available']:
            length,width=obj['measured_size_mm'];label+=f' {length:.1f} x {width:.1f} mm'
        else:label+=': outline unavailable'
        xy=tuple(q.mean(axis=0).astype(int))
        cv2.putText(view,label,xy,cv2.FONT_HERSHEY_SIMPLEX,.55,colour_value,2)
    return view


def live(args,plane,settings):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge
    rclpy.init()
    class Dimensions(Node):
        def __init__(self):
            super().__init__('board_camera_dimensions')
            self.condition=threading.Condition();self.images=OrderedDict();self.info=None;self.latest=None
            self.bridge=CvBridge();group=ReentrantCallbackGroup()
            self.pub=self.create_publisher(String,'/perception/boards_camera',1)
            self.debug=self.create_publisher(Image,'/perception/boards_camera/debug_image',1)
            self.images_sub=self.create_subscription(Image,'/scene/image_raw',self.image,
                qos_profile_sensor_data,callback_group=group)
            self.info_sub=self.create_subscription(CameraInfo,'/scene/camera_info',self.camera,
                qos_profile_sensor_data,callback_group=group)
            self.b_sub=self.create_subscription(String,'/perception/boards_pixels',self.packet,1,
                callback_group=group)
            self.service=self.create_service(Trigger,'/perception/get_boards_camera',self.get,
                callback_group=group)
            self.get_logger().info('READY: camera-frame dimensions only; no robot commands')

        def now(self):return self.get_clock().now().nanoseconds*1e-9

        def image(self,msg):
            with self.condition:
                self.images[(msg.header.stamp.sec,msg.header.stamp.nanosec)]=msg
                while len(self.images)>25:self.images.popitem(last=False)

        def camera(self,msg):
            with self.condition:self.info=msg

        def packet(self,msg):
            try:
                packet=json.loads(msg.data)
                if packet.get('status')!='ok' or not -.1<=self.now()-packet['source_timestamp_sec']<=args.max_age:
                    raise ValueError('B observation is invalid/stale')
                key=(packet['source_stamp']['sec'],packet['source_stamp']['nanosec'])
                with self.condition:
                    source=self.images.get(key);info=self.info
                if source is None or info is None:raise ValueError('Waiting for matching source image and CameraInfo')
                if source.header.frame_id!=plane['camera_frame']:
                    raise ValueError('Wrong image frame')
                if info.header.frame_id not in ('',plane['camera_frame']):
                    raise ValueError('Wrong CameraInfo frame')
                if info.distortion_model!='plumb_bob':raise ValueError('Need raw plumb_bob CameraInfo')
                check_camera(plane['calibration'],[info.width,info.height],info.k,info.d)
                image=self.bridge.imgmsg_to_cv2(source,'bgr8')
                result=measure_packet(packet,image,settings,plane)
                result.update(calibration_id=hashlib.sha256(args.plane.read_bytes()).hexdigest(),
                              observation_age_s=self.now()-packet['source_timestamp_sec'])
                if result['observation_age_s']>args.max_age:raise ValueError('Measurement became stale')
                visual=self.bridge.cv2_to_imgmsg(overlay(image,result),'bgr8');visual.header=source.header
                with self.condition:self.latest=result;self.condition.notify_all()
                self.pub.publish(String(data=json.dumps(result,allow_nan=False)));self.debug.publish(visual)
            except Exception as error:
                with self.condition:self.latest=None;self.condition.notify_all()
                self.get_logger().warning(str(error),throttle_duration_sec=5.)
                self.pub.publish(String(data=json.dumps({'status':'error','error':str(error),
                    'objects':[],'execution_allowed':False})))

        def get(self,request,response):
            after=self.now()
            with self.condition:
                ready=self.condition.wait_for(lambda:self.latest is not None
                    and self.latest['source_timestamp_sec']>=after
                    and -.1<=self.now()-self.latest['source_timestamp_sec']<=args.max_age,timeout=6)
                if not ready:response.success=False;response.message='No fresh camera-frame measurement';return response
                result=copy.deepcopy(self.latest)
            result['observation_age_s']=self.now()-result['source_timestamp_sec']
            response.success=True;response.message=json.dumps(result,allow_nan=False);return response
    node=Dimensions();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def self_test():
    settings={'minimum_colour_fraction':.55,'size_relative_tolerance':.12,
              'maximum_tag_rms_px':1.,'tag_dictionary':'DICT_5X5_100',
              'classes':{'VARa':{'colour':'white','size_m':[.295,.210,.004],'destination_tag_id':1},
                         'VARb':{'colour':'white','size_m':[.210,.152,.004],'destination_tag_id':2},
                         'VARc':{'colour':'red','size_m':[.210,.142,.004],'destination_tag_id':16}},
              'bins':{'1':{'tag_size_m':.1,'T_tag_placement':None}}}
    cal={'image_size':[640,480],'camera_matrix':[[800,0,320],[0,800,240],[0,0,1]],
         'distortion_coefficients':[0.]*5,'distortion_model':'plumb_bob'}
    corners=np.array([[-.295/2,-.210/2,0],[.295/2,-.210/2,0],
                      [.295/2,.210/2,0],[-.295/2,.210/2,0]])
    rv=np.array([.3,.1,.1]);translation=np.array([.1,-.1,1.])
    uv=cv2.projectPoints(corners,rv,translation,np.asarray(cal['camera_matrix']),np.zeros(5))[0].reshape(4,2)
    reference_plane=board_plane(cal,settings,'VARa',uv,2.)
    np.testing.assert_allclose(reference_plane['point_camera_m'],translation,atol=1e-7)
    np.testing.assert_allclose(reference_plane['normal_camera'],-cv2.Rodrigues(rv)[0][:,2],atol=1e-7)
    plane=copy.deepcopy(reference_plane)
    plane.update(point_camera_m=[0,0,1.],normal_camera=[0,0,-1.],board_top_above_reference_m=.004)
    for label,spec in settings['classes'].items():
        length,width=spec['size_m'][:2]
        world=np.array([[-length/2,-width/2],[length/2,-width/2],
                        [length/2,width/2],[-length/2,width/2]])
        uv=world*(800/.996)+[320,240]
        packet={'status':'ok','snapshot_id':'test','source_timestamp_sec':1.,
                'frame_id':plane['camera_frame'],'image_size':[640,480],
                'objects':[{'instance_id':'test:1','supported_pixel_corners':uv.tolist()}]}
        image=np.full((480,640,3),(20,20,180) if spec['colour']=='red' else (180,180,180),np.uint8)
        result=measure_packet(packet,image,settings,plane);obj=result['objects'][0]
        np.testing.assert_allclose(obj['measured_size_m'],[length,width],atol=1e-8)
        np.testing.assert_allclose(obj['centre_camera_m'],[0,0,.996],atol=1e-8)
        assert obj['class_id']==label,(label,obj)
        assert not result['robot_pose_available'] and not result['execution_allowed']
    packet['objects'][0]['supported_pixel_corners']=None
    assert not measure_packet(packet,image,settings,plane)['objects'][0]['measurement_available']
    check_camera(cal,[640,480],cal['camera_matrix'],cal['distortion_coefficients'])
    for size,k in (([320,240],cal['camera_matrix']),([640,480],np.eye(3))):
        try:check_camera(cal,size,k,cal['distortion_coefficients'])
        except ValueError:pass
        else:raise AssertionError('Mismatched camera calibration accepted')
    packet['status']='stale'
    try:measure_packet(packet,image,settings,plane)
    except ValueError:pass
    else:raise AssertionError('Stale packet accepted')
    print('PASS: known-rectangle scale, height correction, three size/colour classes, missing outline and camera mismatch/staleness rejection')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['self-test','calibrate-board','offline','live'])
    p.add_argument('--settings',type=Path,default=Path('config/sorting_task.json'))
    p.add_argument('--calibration',type=Path,default=Path('config/calibration/logitech_extrinsics_candidate_001.json'))
    p.add_argument('--image',type=Path)
    p.add_argument('--reference-class')
    p.add_argument('--max-reference-rms-px',type=float,default=2.)
    p.add_argument('--plane',type=Path)
    p.add_argument('--report',type=Path)
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--confirm-fixed-camera',action='store_true')
    p.add_argument('--single-layer',action='store_true')
    p.add_argument('--max-age',type=float,default=2.)
    a=p.parse_args()
    if a.mode=='self-test':self_test();return
    settings=json.loads(a.settings.read_text())
    if a.plane is None:p.error('--plane is required')
    if a.mode=='calibrate-board':
        if a.image is None:p.error('Need a reference image')
        if a.plane.exists():p.error('Preserve the existing plane; choose a new --plane filename')
        cal=json.loads(a.calibration.read_text())
        image=cv2.imread(str(a.image))
        if image is None or [image.shape[1],image.shape[0]]!=cal['image_size']:
            p.error('Need an original-resolution reference image matching calibration')
        if cal.get('distortion_model')!='plumb_bob':p.error('Need raw Logitech plumb_bob calibration')
        if a.reference_class not in settings['classes'] or not a.single_layer:
            p.error('Choose reference-class and confirm one flat board with --single-layer')
        if not np.isfinite(a.max_reference_rms_px) or a.max_reference_rms_px<=0:
            p.error('Reference RMS limit must be positive')
        result=board_plane(cal,settings,a.reference_class,click_reference(image),a.max_reference_rms_px)
        result.update(reference_image=str(a.image.resolve()),
                      reference_sha256=hashlib.sha256(a.image.read_bytes()).hexdigest())
        a.plane.parent.mkdir(parents=True,exist_ok=True)
        with a.plane.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
        print(json.dumps(result,indent=2));print('Saved:',a.plane);return
    if not a.confirm_fixed_camera or not a.single_layer:
        p.error('Confirm unchanged camera/table and a separated single-layer test scene')
    if not np.isfinite(a.max_age) or a.max_age<=0:p.error('max-age must be positive')
    plane=json.loads(a.plane.read_text())
    if a.mode=='live':live(a,plane,settings);return
    if a.report is None or a.output_dir is None:p.error('offline needs --report and --output-dir')
    from b_live_geometry import measure
    report=json.loads(a.report.read_text());image=cv2.imread(report['source_image'])
    objects=[]
    for candidate in report['quad_candidates']:
        mask=cv2.imread(str(a.report.parent/candidate['mask_file']),cv2.IMREAD_GRAYSCALE)
        if mask is None:raise ValueError('Missing B instance mask')
        obj=measure(candidate,mask);obj['instance_id']='offline:'+candidate['id'];objects.append(obj)
    packet={'status':'offline','snapshot_id':'offline:'+hashlib.sha256(a.report.read_bytes()).hexdigest()[:12],
            'frame_id':plane['camera_frame'],'image_size':report['image_size'],'objects':objects}
    result=measure_packet(packet,image,settings,plane)
    a.output_dir.mkdir(parents=True,exist_ok=False)
    (a.output_dir/'measurements.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    if not cv2.imwrite(str(a.output_dir/'overlay.jpg'),overlay(image,result)):
        raise OSError('Cannot save measurement overlay')
    print(json.dumps(result,indent=2));print('Saved:',a.output_dir)


if __name__=='__main__':main()
