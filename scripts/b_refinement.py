"""Conservative RGB-only board proposals. Does not infer hidden edges or layer order."""
import cv2
import numpy as np
from b_live_geometry import supported_quad


def separator_masks(image,mask):
    """Split hypotheses only at long observed interior lines, never invented seams."""
    ys,xs=np.where(mask)
    if len(xs)<100:return []
    x0,y0,x1,y1=xs.min(),ys.min(),xs.max()+1,ys.max()+1
    local=mask[y0:y1,x0:x1]
    gray=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY)
    lines=cv2.createLineSegmentDetector().detect(gray)[0]
    if lines is None:return []
    interior=cv2.erode(local.astype(np.uint8),np.ones((9,9),np.uint8))>0
    yy,xx=np.indices(local.shape);parts=[]
    short=min(cv2.minAreaRect(np.column_stack([xs,ys]).astype(np.float32))[1])
    for segment in lines[:,0]:
        a=segment[:2];b=segment[2:];v=b-a;length=np.linalg.norm(v)
        if length<max(25,short*.65):continue
        samples=np.rint(a[None,:]+np.linspace(.1,.9,30)[:,None]*v).astype(int)
        samples[:,0]=np.clip(samples[:,0],0,local.shape[1]-1)
        samples[:,1]=np.clip(samples[:,1],0,local.shape[0]-1)
        if interior[samples[:,1],samples[:,0]].mean()<.8:continue
        side=(xx-a[0])*v[1]-(yy-a[1])*v[0]
        halves=[local & (side>2*length),local & (side< -2*length)]
        if min(int(p.sum()) for p in halves)<.18*local.sum():continue
        for half in halves:
            full=np.zeros_like(mask);full[y0:y1,x0:x1]=half
            if supported_quad(full) is not None and not any(same_instance(full,p) for p in parts):parts.append(full)
        if len(parts)>=4:break
    return parts


def rectangle_check(image,mask):
    q=supported_quad(mask,allow_rectangle_fit=True)
    if q is None:return {'passed':False,'reason':'Visible mask is not a supported quadrilateral'}
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    edges=cv2.Canny(gray,40,100)
    nearby=cv2.dilate(edges,np.ones((7,7),np.uint8))>0
    supports=[]
    for a,b in zip(q,np.roll(q,-1,axis=0)):
        # Ignore corner pixels, where a crossing edge can give misleading support.
        t=np.linspace(.08,.92,max(12,int(np.linalg.norm(b-a))))
        p=np.rint(a[None,:]+t[:,None]*(b-a)[None,:]).astype(int)
        p[:,0]=np.clip(p[:,0],0,mask.shape[1]-1)
        p[:,1]=np.clip(p[:,1],0,mask.shape[0]-1)
        supports.append(float(nearby[p[:,1],p[:,0]].mean()))
    passed=min(supports)>=.5 and np.mean(supports)>=.7
    seam=bool(separator_masks(image,mask)) if passed else False
    passed=passed and not seam
    return {'passed':bool(passed),'reason':('Possible internal board separator' if seam else
            'Supported visible quadrilateral' if passed else 'Insufficient image-edge support'),
            'edge_support':supports}


def interior_seeds(mask,limit=3):
    distance=cv2.distanceTransform(mask.astype(np.uint8),cv2.DIST_L2,5)
    seeds=[]
    for _ in range(limit):
        _,value,_,point=cv2.minMaxLoc(distance)
        if value<5:break
        seeds.append(list(point))
        cv2.circle(distance,point,max(16,int(value*2)),0,-1)
    return seeds


def prompts_for_masks(masks,checks,limit=6,image=None):
    prompts=[]
    for i,mask in enumerate(masks):
        if checks[i]['passed']:continue
        if image is not None:
            for part in separator_masks(image,mask):
                positive=interior_seeds(part,1);negative=interior_seeds(mask & ~part,1)
                if positive and negative:
                    prompts.append({'source_index':i,'points':[positive[0],negative[0]],
                                    'labels':[1,0],'seed_rank':-1,'evidence':'observed_internal_line'})
        for rank,point in enumerate(interior_seeds(mask)):
            points=[point];labels=[1]
            # Only use another proposal's exclusive surface as an exclusion cue.
            # Other seeds in this same merged mask are NOT known to be another board.
            for j,other in enumerate(masks):
                if j==i:continue
                candidates=interior_seeds(other & ~mask,1)
                if candidates:
                    points.append(candidates[0]);labels.append(0)
                    if len(points)==3:break
            prompts.append({'source_index':i,'points':points,'labels':labels,'seed_rank':rank})
    # Round robin prevents the first merged proposal consuming the entire budget.
    prompts.sort(key=lambda p:p['seed_rank'])
    return prompts[:limit]


def same_instance(a,b):
    intersection=np.count_nonzero(a & b)
    union=np.count_nonzero(a | b)
    smaller=min(np.count_nonzero(a),np.count_nonzero(b))
    return bool(union and (intersection/union>.85 or (smaller and intersection/smaller>.95)))


def select_consistent(image,candidates):
    """Candidates are dictionaries with mask/quality; no semantic layer-order claim."""
    accepted=[];rejected=[]
    for item in sorted(candidates,key=lambda c:-c['quality']):
        check=rectangle_check(image,item['mask'])
        item={**item,'rectangle_consistency':check}
        if not check['passed']:
            rejected.append((item,check['reason']));continue
        duplicate=any(same_instance(item['mask'],p['mask']) for p in accepted)
        if duplicate:
            rejected.append((item,'Duplicate or nested visible-mask hypothesis'));continue
        # Two physical opaque board surfaces cannot occupy the same visible pixels.
        conflict=any(np.count_nonzero(item['mask'] & p['mask'])/
                     min(np.count_nonzero(item['mask']),np.count_nonzero(p['mask']))>.1
                     for p in accepted)
        if conflict:
            rejected.append((item,'Conflicting visible instances; layer order unresolved'));continue
        accepted.append(item)
    return accepted,rejected
