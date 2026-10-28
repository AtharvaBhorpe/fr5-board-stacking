"""Image-only mask geometry; a projected board need not be an image rectangle."""
import cv2
import numpy as np
from b_pixels import rectangle_geometry


def quad_center_and_axis(points):
    q=np.asarray(points,np.float64).reshape(4,2)
    a=np.column_stack([q[2]-q[0],-(q[3]-q[1])])
    if not np.isfinite(q).all() or abs(np.linalg.det(a))<1e-6:
        raise ValueError('Degenerate quadrilateral')
    t,s=np.linalg.solve(a,q[1]-q[0])
    if not (0<t<1 and 0<s<1):raise ValueError('Quadrilateral diagonals do not cross internally')
    centre=q[0]+t*(q[2]-q[0])
    edges=np.roll(q,-1,axis=0)-q
    lengths=np.linalg.norm(edges,axis=1)
    h=cv2.getPerspectiveTransform(np.float32([[0,0],[1,0],[1,1],[0,1]]),q.astype(np.float32))
    if lengths[0]+lengths[2]>=lengths[1]+lengths[3]:
        endpoints=np.float64([[[0,.5],[1,.5]]])
    else:endpoints=np.float64([[[.5,0],[.5,1]]])
    axis=cv2.perspectiveTransform(endpoints,h)[0]
    vector=axis[1]-axis[0]
    angle=float(np.degrees(np.arctan2(vector[1],vector[0]))%180)
    return centre,angle


def supported_quad(mask,allow_rectangle_fit=False):
    binary=(np.asarray(mask)>0).astype(np.uint8)
    x,y,w,h=cv2.boundingRect(binary)
    if not w or not h:return None
    observed=binary[y:y+h,x:x+w]>0
    contours,_=cv2.findContours(observed.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours)!=1:
        count,labels,stats,_=cv2.connectedComponentsWithStats(observed.astype(np.uint8))
        if count<2:return None
        largest=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]))
        # Ignore at most 0.5% satellite pixels; retain the original mask in fit scoring.
        if 1-float(stats[largest,cv2.CC_STAT_AREA])/np.count_nonzero(observed)>.005:return None
        contours,_=cv2.findContours((labels==largest).astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    c=contours[0];length=cv2.arcLength(c,True)
    options=[cv2.approxPolyDP(c,epsilon*length,True) for epsilon in (.01,.015,.02)]
    if allow_rectangle_fit:
        options.append(np.rint(cv2.boxPoints(cv2.minAreaRect(c))).astype(np.int32))
    for q in options:
        if len(q)!=4 or not cv2.isContourConvex(q):continue
        q=q.reshape(4,2)
        # A legacy enclosing-rectangle fallback can extend beyond the mask bbox.
        full_score=bool(np.any(q<0) or np.any(q[:,0]>=w) or np.any(q[:,1]>=h))
        reference=binary>0 if full_score else observed
        approximated=np.zeros(reference.shape,np.uint8)
        cv2.fillConvexPoly(approximated,q+[x,y] if full_score else q,1)
        predicted=approximated>0
        union=np.count_nonzero(reference|predicted)
        if union and np.count_nonzero(reference&predicted)/union>=.93:
            return q.astype(float)+[x,y]
    return None


def measure(candidate,mask=None):
    rectangle=rectangle_geometry(candidate['pixel_corners'])
    result={'candidate_id':candidate['id'],'rectangle_center_px':rectangle['rectangle_center_px'],
            'mask_centroid_px':candidate['pixel_center'],
            'enclosing_rectangle_corners':candidate['pixel_corners'],
            'enclosing_rectangle_angle_deg':rectangle['long_axis_angle_deg'],
            'centre_px':rectangle['rectangle_center_px'],
            'orientation_image_deg':rectangle['long_axis_angle_deg'],
            'supported_pixel_corners':None,'outline_kind':'enclosing_rectangle_only',
            'text_prompt_label':candidate.get('text_label'),
            'detector_score':candidate.get('detector_score'),
            'near_square_orientation_ambiguous':rectangle['near_square_orientation_ambiguous'],
            'instance_isolation_verified':False,'execution_allowed':False}
    result.update(mask_origin=candidate.get('mask_origin','box_prompt'),
                  class_name=candidate.get('class_name'),class_id=candidate.get('class_id'),
                  confidence=candidate.get('confidence'),
                  rectangle_consistency=candidate.get('rectangle_consistency'),layer_order_verified=False,
                  detection_status=candidate.get('detection_status','candidate'),
                  uncertainty_reason=candidate.get('uncertainty_reason'))
    checked=bool((candidate.get('rectangle_consistency') or {}).get('passed'))
    q=candidate.get('supported_mask_quad')
    if q is not None:
        q=np.asarray(q,float)
        if q.shape!=(4,2) or not np.isfinite(q).all() or not cv2.isContourConvex(q.astype(np.float32)):
            raise ValueError('Invalid cached supported quadrilateral')
    elif mask is not None:q=supported_quad(mask,allow_rectangle_fit=checked)
    if q is not None:
        centre,angle=quad_center_and_axis(q)
        result.update(centre_px=centre.tolist(),orientation_image_deg=angle,
                      supported_pixel_corners=q.tolist(),outline_kind='supported_mask_quadrilateral')
    return result


def draw(image,objects):
    for item in objects:
        is_quad=item['supported_pixel_corners'] is not None and item.get('detection_status')!='uncertain'
        points=item['supported_pixel_corners'] if is_quad else item['enclosing_rectangle_corners']
        colour=(0,255,0) if is_quad else (0,165,255)
        cv2.polylines(image,[np.rint(points).astype(np.int32)],True,colour,2)
        centre=np.asarray(item['centre_px'])
        angle=np.radians(item['orientation_image_deg'])
        start=tuple(np.rint(centre).astype(int))
        end=tuple(np.rint(centre+60*np.array([np.cos(angle),np.sin(angle)])).astype(int))
        cv2.drawMarker(image,start,colour,cv2.MARKER_CROSS,16,2)
        cv2.arrowedLine(image,start,end,colour,2)
        label=f'{item["candidate_id"]}: {centre[0]:.0f},{centre[1]:.0f} {item["orientation_image_deg"]:.1f}deg'
        if item.get('class_name'):label=item['class_name']+' '+label
        if item.get('detection_status')=='uncertain':label='UNCERTAIN '+label
        cv2.putText(image,label,start,cv2.FONT_HERSHEY_SIMPLEX,.5,colour,2)
    return image


def self_test():
    q=np.array([[40,30],[250,50],[210,170],[55,145]],np.int32)
    mask=np.zeros((210,300),np.uint8);cv2.fillConvexPoly(mask,q,255)
    found=supported_quad(mask)
    assert found is not None
    centre,angle=quad_center_and_axis(q)
    h=cv2.getPerspectiveTransform(np.float32([[0,0],[1,0],[1,1],[0,1]]),q.astype(np.float32))
    expected=cv2.perspectiveTransform(np.float64([[[.5,.5]]]),h)[0,0]
    np.testing.assert_allclose(centre,expected,atol=1e-6)
    assert np.linalg.norm(centre-q.mean(axis=0))>1
    mask[80:130,150:290]=0
    assert supported_quad(mask) is None
    print('PASS: perspective quadrilateral, projective centre, occluded-outline rejection')
