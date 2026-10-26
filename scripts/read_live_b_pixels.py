"""Request fresh B image coordinates for a motion-side consumer; never move a robot."""
import argparse
import json
import math


def handoff(packet,now,max_age=2.):
    if packet.get('status')!='ok' or packet.get('coordinate_system')!='raw_image_pixels':
        raise ValueError('Need a valid raw-image observation')
    if packet.get('frame_id')!='scene_camera_optical_frame':
        raise ValueError('Unexpected camera frame')
    stamp=float(packet['source_timestamp_sec']);age=now-stamp
    if not math.isfinite(age) or not -.1<=age<=max_age:
        raise ValueError('Observation stale or clocks are not synchronized')
    width,height=packet['image_size']
    if not all(type(v) is int and v>0 for v in (width,height)):
        raise ValueError('Invalid image dimensions')
    objects=[]
    for obj in packet['objects']:
        u,v=map(float,obj['centre_px']);angle=float(obj['orientation_image_deg'])
        if not all(math.isfinite(x) for x in (u,v,angle)) or not (0<=u<width and 0<=v<height):
            raise ValueError('Invalid pixel coordinates')
        objects.append({'instance_id':obj['instance_id'],'candidate_id':obj['candidate_id'],
                        'centre_px':[u,v],'orientation_image_deg':angle,
                        'outline_kind':obj['outline_kind'],
                        'supported_pixel_corners':obj.get('supported_pixel_corners'),
                        'text_prompt_label':obj.get('text_prompt_label'),
                        'instance_isolation_verified':False})
    if len({o['instance_id'] for o in objects})!=len(objects):
        raise ValueError('Duplicate instance IDs')
    return {'snapshot_id':packet['snapshot_id'],'source_timestamp_sec':stamp,
            'source_stamp':packet['source_stamp'],'observation_age_s':age,
            'frame_id':packet['frame_id'],'coordinate_system':'raw_image_pixels',
            'image_size':[width,height],'objects':objects,'execution_allowed':False}


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
