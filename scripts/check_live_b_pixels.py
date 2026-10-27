"""Record and independently review live B pixel observations. Never commands motion."""
import argparse
from collections import OrderedDict
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np
from b_live_geometry import quad_center_and_axis
from read_live_b_pixels import handoff


def match_centres(reference,predicted,radius=float('inf')):
    """Small-set assignment: maximize valid matches, then minimize total distance."""
    if len(reference)>8 or len(predicted)>8:raise ValueError('This test supports up to eight boards')
    a=np.asarray(reference,float).reshape(-1,2);b=np.asarray(predicted,float).reshape(-1,2)
    if not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('Invalid centres')
    costs=np.linalg.norm(a[:,None,:]-b[None,:,:],axis=2)
    @lru_cache(None)
    def solve(i,used):
        if i==len(a):return 0,0.,()
        best=solve(i+1,used)
        for j in range(len(b)):
            if used&(1<<j) or costs[i,j]>radius:continue
            count,cost,pairs=solve(i+1,used|(1<<j))
            candidate=(count+1,cost+float(costs[i,j]),((i,j),)+pairs)
            if candidate[0]>best[0] or (candidate[0]==best[0] and candidate[1]<best[1]):best=candidate
        return best
    return list(solve(0,0)[2])


def angle_error(a,b):return abs((a-b+90)%180-90)


def stability(packets,expected_count,max_jitter,max_angle):
    if expected_count==0:return {'applicable':False}
    if len(packets)<3 or any(len(p['objects'])!=expected_count for p in packets):
        return {'applicable':False,'reason':'Need at least three frames with the expected count'}
    baseline=packets[0]['objects'];positions=[[] for _ in baseline];angles=[[] for _ in baseline]
    for packet in packets:
        pairs=match_centres([o['centre_px'] for o in baseline],[o['centre_px'] for o in packet['objects']])
        for i,j in pairs:
            positions[i].append(packet['objects'][j]['centre_px'])
            angles[i].append(packet['objects'][j]['orientation_image_deg'])
    tracks=[]
    for i,(xy,theta) in enumerate(zip(positions,angles)):
        xy=np.asarray(xy,float);median=np.median(xy,axis=0)
        radius=np.linalg.norm(xy-median,axis=1)
        delta=np.array([(v-theta[0]+90)%180-90 for v in theta])
        angular=np.abs(delta-np.median(delta))
        tracks.append({'test_association_index':i+1,'median_centre_px':median.tolist(),
                       'p95_centre_jitter_px':float(np.percentile(radius,95)),
                       'maximum_centre_jitter_px':float(radius.max()),
                       'p95_angle_jitter_deg':float(np.percentile(angular,95))})
    return {'applicable':True,'objects':tracks,
            'passed':all(t['p95_centre_jitter_px']<=max_jitter and
                         t['p95_angle_jitter_deg']<=max_angle for t in tracks),
            'limits':'Nearest geometric association for a stationary scene; B IDs are not persistent tracks.'}


def capture(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge
    if args.output_dir.exists():raise ValueError('Preserve existing results; choose a new output-dir')
    args.output_dir.mkdir(parents=True)
    rclpy.init();node=Node('check_live_b_pixels');bridge=CvBridge()
    raw=OrderedDict();overlays=OrderedDict()
    def receive(buffer,msg):
        buffer[(msg.header.stamp.sec,msg.header.stamp.nanosec)]=msg
        while len(buffer)>25:buffer.popitem(last=False)
    source_sub=node.create_subscription(Image,'/scene/image_raw',lambda m:receive(raw,m),qos_profile_sensor_data)
    overlay_sub=node.create_subscription(Image,'/perception/boards_pixels/debug_image',
                                         lambda m:receive(overlays,m),qos_profile_sensor_data)
    client=node.create_client(Trigger,'/perception/get_boards_pixels')
    def until(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return True
        return False
    packets=[];attempts=[]
    try:
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('B service missing; wait for READY')
        for i in range(args.samples):
            folder=args.output_dir/f'sample_{i+1:03}';folder.mkdir()
            attempt={'sample':i+1,'expected_count':args.expected_count,'ok':False}
            try:
                requested=node.get_clock().now().nanoseconds*1e-9
                start=time.monotonic();future=client.call_async(Trigger.Request())
                if not until(future.done,10):raise RuntimeError('Service request timed out')
                received=node.get_clock().now().nanoseconds*1e-9
                response=future.result()
                if not response.success:raise RuntimeError(response.message)
                packet=json.loads(response.message)
                (folder/'packet.json').write_text(json.dumps(packet,indent=2,allow_nan=False))
                cleaned=handoff(packet,received,args.max_age_s)
                if packet['source_timestamp_sec']+1e-6<requested:
                    raise ValueError('Service returned an image older than the request')
                stamp=packet['source_stamp'];key=(stamp['sec'],stamp['nanosec'])
                if not until(lambda:key in raw and key in overlays,2):
                    raise ValueError('Cannot match raw image and overlay to this exact source stamp')
                original=raw[key];visual=overlays[key]
                if original.header.frame_id!=packet['frame_id'] or visual.header.frame_id!=packet['frame_id']:
                    raise ValueError('Image/overlay frame mismatch')
                source=bridge.imgmsg_to_cv2(original,'bgr8');overlay=bridge.imgmsg_to_cv2(visual,'bgr8')
                if list(source.shape[:2][::-1])!=packet['image_size'] or overlay.shape!=source.shape:
                    raise ValueError('Image/overlay resolution mismatch')
                if not cv2.imwrite(str(folder/'source.png'),source) or not cv2.imwrite(str(folder/'overlay.jpg'),overlay):
                    raise OSError('Evidence image save failed')
                (folder/'handoff.json').write_text(json.dumps(cleaned,indent=2,allow_nan=False))
                supported=sum(o.get('supported_pixel_corners') is not None for o in packet['objects'])
                attempt.update(ok=True,received_sec=received,source_timestamp_sec=packet['source_timestamp_sec'],
                    request_sent_sec=requested,source_age_at_receive_s=received-packet['source_timestamp_sec'],
                    service_round_trip_s=time.monotonic()-start,
                    count=len(packet['objects']),supported_outline_count=supported,
                    count_matches=len(packet['objects'])==args.expected_count,
                    exact_image_stamp_matched=True)
                packets.append(packet)
                print(f"{i+1}/{args.samples}: count={attempt['count']}, supported={supported}, age={attempt['source_age_at_receive_s']:.3f}s",flush=True)
            except Exception as error:
                attempt['error']=str(error);print(f'{i+1}/{args.samples}: FAIL {error}',flush=True)
            (folder/'test.json').write_text(json.dumps(attempt,indent=2,allow_nan=False));attempts.append(attempt)
        ages=[v['source_age_at_receive_s'] for v in attempts if v['ok']]
        result={'samples_requested':args.samples,'valid_samples':len(packets),
                'transport_passed':len(packets)==args.samples,
                'count_checks_passed':len(packets)==args.samples and all(v.get('count_matches') for v in attempts),
                'all_outlines_supported':len(packets)==args.samples and all(
                    v.get('supported_outline_count')==args.expected_count for v in attempts),
                'source_age_p95_s':float(np.percentile(ages,95)) if ages else None,
                'stationary_scene_confirmed_by_operator':args.stationary,
                'stability':stability(packets,args.expected_count,args.max_jitter_px,args.max_angle_jitter_deg)
                            if args.stationary else {'applicable':False},
                'visual_accuracy_validated':False,'execution_allowed':False,
                'limits':['Count equality alone is not precision/recall or proof of correct board instances.',
                          'Saved coordinates are historical evidence; actual picks need a new live request.',
                          'Pixel tests do not validate robot coordinates or collision avoidance.'],
                'attempts':attempts}
        (args.output_dir/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False))
        print(json.dumps({k:v for k,v in result.items() if k!='attempts'},indent=2))
        print('Saved:',args.output_dir)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def mark(args):
    source=args.sample/'source.png';image=cv2.imread(str(source))
    if image is None:raise ValueError('Missing captured source.png')
    count=args.boards
    if count is None:count=json.loads((args.sample/'test.json').read_text())['expected_count']
    if not 1<=count<=8:raise ValueError('Mark one to eight fully visible boards')
    output=args.reference or args.sample/'reference.json'
    if output.exists():raise ValueError('Reference exists; use a new --reference filename')
    scale=min(1.,1600/image.shape[1],900/image.shape[0]);clicks=[]
    window='Independent reference: RAW image; four corners around each visible board'
    cv2.namedWindow(window)
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(clicks)<4*count:
            clicks.append([(x+.5)/scale-.5,(y+.5)/scale-.5])
    cv2.setMouseCallback(window,click)
    try:
        while True:
            view=cv2.resize(image,None,fx=scale,fy=scale)
            for i,q in enumerate(clicks):
                xy=tuple(np.rint(np.asarray(q)*scale).astype(int))
                cv2.circle(view,xy,4,(0,255,0),2)
                cv2.putText(view,f'GT{i//4+1}:{i%4+1}',xy,cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
            text=f'Board {min(len(clicks)//4+1,count)}: click corners in perimeter order; U undo; Enter save'
            cv2.putText(view,text,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
            cv2.imshow(window,view);key=cv2.waitKey(30)&255
            if key==27:return
            if key in (ord('u'),ord('U')) and clicks:clicks.pop()
            if key in (10,13) and len(clicks)==4*count:
                boards=[]
                for i in range(count):
                    q=clicks[4*i:4*i+4];centre,angle=quad_center_and_axis(q)
                    boards.append({'reference_id':f'GT{i+1}','corners_px':q,
                                   'centre_px':centre.tolist(),'orientation_image_deg':angle})
                with output.open('x') as stream:json.dump({'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                    'boards':boards,'scope':'Human-clicked fully visible board corners, independent of model overlay'},stream,indent=2)
                print('Saved:',output);return
    finally:cv2.destroyAllWindows()


def accuracy(packet,reference,max_centre=5.,max_angle=5.,max_corner=8.,match_radius=50.):
    gt=reference['boards'];objects=packet['objects']
    pairs=match_centres([g['centre_px'] for g in gt],[o['centre_px'] for o in objects],match_radius)
    results=[]
    for i,j in pairs:
        g,o=gt[i],objects[j]
        error=float(np.linalg.norm(np.asarray(g['centre_px'])-o['centre_px']))
        angle=angle_error(g['orientation_image_deg'],o['orientation_image_deg'])
        corner=None
        if o.get('supported_pixel_corners') is not None:
            q=np.asarray(o['supported_pixel_corners'],float);target=np.asarray(g['corners_px'],float)
            corner=min(float(np.sqrt(np.mean(np.sum((np.roll(q,k,axis=0)-t)**2,axis=1))))
                       for t in (target,target[::-1]) for k in range(4))
        results.append({'reference_id':g['reference_id'],'instance_id':o['instance_id'],
                        'centre_error_px':error,'angle_error_deg':angle,'corner_rms_px':corner,
                        'passed':error<=max_centre and angle<=max_angle and corner is not None and corner<=max_corner})
    unmatched_gt=[g['reference_id'] for i,g in enumerate(gt) if i not in {p[0] for p in pairs}]
    unmatched_pred=[o['instance_id'] for j,o in enumerate(objects) if j not in {p[1] for p in pairs}]
    return {'matched':results,'unmatched_reference_ids':unmatched_gt,'unmatched_prediction_ids':unmatched_pred,
            'passed':not unmatched_gt and not unmatched_pred and all(r['passed'] for r in results),
            'limits':'Human-corner pixel check with centre-radius matching; not mask IoU, physical grasp accuracy or calibration.'}


def score(args):
    reference=args.reference or args.sample/'reference.json'
    ref=json.loads(reference.read_text())
    if ref['source_sha256']!=hashlib.sha256((args.sample/'source.png').read_bytes()).hexdigest():
        raise ValueError('Reference belongs to a different source image')
    packet=json.loads((args.sample/'packet.json').read_text())
    result=accuracy(packet,ref,args.max_centre_error_px,args.max_angle_error_deg,args.max_corner_rms_px,args.match_radius_px)
    output=args.output or args.sample/'accuracy.json'
    with output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2));print('Saved:',output)


def unavailable():
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init();node=rclpy.create_node('check_b_camera_interruption')
    try:
        client=node.create_client(Trigger,'/perception/get_boards_pixels')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('B itself is absent; this is not a camera-stop test')
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done():raise RuntimeError('Service did not respond; test inconclusive')
        response=future.result()
        result={'camera_stop_rejected_fresh_request':not response.success,'message':response.message}
        print(json.dumps(result,indent=2))
        if response.success:raise RuntimeError('FAIL: fresh request succeeded after camera was stopped')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def self_test():
    assert match_centres([[0,0],[100,0]],[[101,0],[1,0]])==[(0,1),(1,0)]
    assert match_centres([[0,0]],[[100,0]],50)==[]
    assert angle_error(179,1)==2
    quad=[[100,100],[300,100],[300,200],[100,200]]
    obj={'instance_id':'s:1','centre_px':[200,150],'orientation_image_deg':0.,'supported_pixel_corners':quad}
    p={'objects':[obj]};ref={'boards':[{'reference_id':'GT1','centre_px':[200,150],
                                     'orientation_image_deg':0.,'corners_px':quad}]}
    assert accuracy(p,ref)['passed']
    shifted=json.loads(json.dumps(p));shifted['objects'][0]['centre_px'][0]+=10
    assert not accuracy(shifted,ref)['passed']
    extra={'objects':[obj,{**obj,'instance_id':'s:2'}]}
    assert len(accuracy(extra,ref)['unmatched_prediction_ids'])==1
    assert accuracy({'objects':[]},ref)['unmatched_reference_ids']==['GT1']
    assert stability([p,p,p],1,5,3)['passed']
    assert not stability([p,shifted,p],1,5,3)['passed']
    print('PASS: one-to-one matching, unmatched instances, angle wrap, pixel error and stationary jitter checks')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['self-test','capture','mark','score','expect-unavailable'])
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--expected-count',type=int)
    p.add_argument('--samples',type=int,default=15)
    p.add_argument('--stationary',action='store_true')
    p.add_argument('--max-age-s',type=float,default=2.)
    p.add_argument('--max-jitter-px',type=float,default=5.)
    p.add_argument('--max-angle-jitter-deg',type=float,default=3.)
    p.add_argument('--sample',type=Path)
    p.add_argument('--boards',type=int)
    p.add_argument('--reference',type=Path)
    p.add_argument('--output',type=Path)
    p.add_argument('--max-centre-error-px',type=float,default=5.)
    p.add_argument('--max-angle-error-deg',type=float,default=5.)
    p.add_argument('--max-corner-rms-px',type=float,default=8.)
    p.add_argument('--match-radius-px',type=float,default=50.)
    a=p.parse_args()
    if a.mode=='self-test':self_test();return
    if a.mode=='expect-unavailable':unavailable();return
    if a.mode=='capture':
        if a.output_dir is None or a.expected_count is None or not 0<=a.expected_count<=8 or a.samples<1:
            p.error('capture needs output-dir, expected-count (0-8) and positive samples')
        capture(a);return
    if a.sample is None:p.error('mark/score need --sample')
    if a.mode=='mark':mark(a)
    else:score(a)


if __name__=='__main__':main()
