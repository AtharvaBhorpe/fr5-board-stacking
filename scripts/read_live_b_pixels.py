"""Request fresh B image coordinates for a motion-side consumer; never move a robot."""
import argparse
import json
import math


def handoff(packet,now,max_age=2.):
    if not math.isfinite(max_age) or max_age<=0:
        raise ValueError('max-age must be finite and positive')
    if packet.get('status')!='ok' or packet.get('coordinate_system')!='raw_image_pixels':
        raise ValueError('Need a valid raw-image observation')
    if packet.get('frame_id')!='scene_camera_optical_frame':
        raise ValueError('Unexpected camera frame')
    stamp=float(packet['source_timestamp_sec']);age=now-stamp
    if not math.isfinite(age) or not -.1<=age<=max_age:
        raise ValueError('Observation stale or clocks are not synchronized')
    source=packet['source_stamp']
    if (type(source.get('sec')) is not int or type(source.get('nanosec')) is not int
            or not 0<=source['nanosec']<1_000_000_000
            or abs(stamp-(source['sec']+source['nanosec']*1e-9))>1e-6):
        raise ValueError('Source timestamp fields disagree')
    if not isinstance(packet.get('snapshot_id'),str) or not packet['snapshot_id']:
        raise ValueError('Missing snapshot identity')
    width,height=packet['image_size']
    if not all(type(v) is int and v>0 for v in (width,height)):
        raise ValueError('Invalid image dimensions')
    objects=[]
    for obj in packet['objects']:
        if obj.get('class_name')=='hand':raise ValueError('Hands must not be included in board coordinates')
        u,v=map(float,obj['centre_px']);angle=float(obj['orientation_image_deg'])
        if not all(math.isfinite(x) for x in (u,v,angle)) or not (0<=u<width and 0<=v<height):
            raise ValueError('Invalid pixel coordinates')
        if not 0<=angle<180:
            raise ValueError('Image orientation must be in [0, 180) degrees')
        if obj.get('detection_status')=='uncertain':
            raise ValueError('Uncertain instances must not be included in board coordinates')
        if not isinstance(obj.get('instance_id'),str) or not obj['instance_id']:
            raise ValueError('Missing instance identity')
        confidence=obj.get('confidence')
        if confidence is not None and (not math.isfinite(confidence) or not 0<=confidence<=1):
            raise ValueError('Invalid detection confidence')
        objects.append({'instance_id':obj['instance_id'],'candidate_id':obj['candidate_id'],
                        'centre_px':[u,v],'orientation_image_deg':angle,
                        'outline_kind':obj['outline_kind'],
                        'supported_pixel_corners':obj.get('supported_pixel_corners'),
                        'text_prompt_label':obj.get('text_prompt_label'),
                        'class_name':obj.get('class_name'),'class_id':obj.get('class_id'),
                        'confidence':obj.get('confidence'),
                        'mask_origin':obj.get('mask_origin','box_prompt'),
                        'rectangle_consistency':obj.get('rectangle_consistency'),
                        'near_square_orientation_ambiguous':obj.get('near_square_orientation_ambiguous',False),
                        'layer_order_verified':False,
                        'instance_isolation_verified':False})
    if len({o['instance_id'] for o in objects})!=len(objects):
        raise ValueError('Duplicate instance IDs')
    return {'snapshot_id':packet['snapshot_id'],'source_timestamp_sec':stamp,
            'source_stamp':packet['source_stamp'],'observation_age_s':age,
            'frame_id':packet['frame_id'],'coordinate_system':'raw_image_pixels',
            'model_backend':packet.get('model_backend','grounded_sam2'),
            'checkpoint_sha256':packet.get('checkpoint_sha256'),
            'angle_convention':'clockwise image angle from +u/right, modulo 180 degrees',
            'units':{'centre_px':'pixels','orientation_image_deg':'degrees'},
            'hand_count':len(packet.get('hand_detections',[])),
            'human_safety_validated':False,
            'image_size':[width,height],'objects':objects,
            'geometry_image_size':packet.get('geometry_image_size',[width,height]),
            'geometry_pixel_scale':packet.get('geometry_pixel_scale',[1.,1.]),
            'uncertain_count':len(packet.get('uncertain_objects',[])),'execution_allowed':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-age-s',type=float,default=2.)
    args=parser.parse_args()
    if not math.isfinite(args.max_age_s) or args.max_age_s<=0:
        parser.error('max-age-s must be finite and positive')
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init();node=rclpy.create_node('read_live_b_pixels')
    try:
        client=node.create_client(Trigger,'/perception/get_boards_pixels')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Live B service unavailable; wait for its READY message')
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done():raise RuntimeError('Fresh-observation request timed out')
        response=future.result()
        if not response.success:raise RuntimeError(response.message)
        packet=json.loads(response.message)
        result=handoff(packet,node.get_clock().now().nanoseconds*1e-9,args.max_age_s)
        print(json.dumps(result,indent=2,allow_nan=False))
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
