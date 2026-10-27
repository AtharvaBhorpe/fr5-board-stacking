"""Local browser batch editor for a CVAT YOLO review plan. No uploads or training."""
import argparse
import copy
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import secrets
from urllib.parse import urlparse

from prepare_yolo_pilot import digest, read_export, validate_labels,validate_frozen_reviews


def load_plan(source, plan_path):
    plan=json.loads(plan_path.read_text())
    if plan['source_zip_sha256']!=digest(source):
        raise ValueError('ZIP differs from review plan; create an inventory for this export first')
    images,labels,names=read_export(source)
    seen=set()
    for row in plan['images']:
        name=row['file_name']
        if name not in images or name in seen:raise ValueError('Unknown or duplicate image: '+name)
        seen.add(name)
        _,counts=validate_labels(labels.get(Path(name).stem,''),names)
        row['exported_instances']=sum(counts.values())
    return plan,images,labels,names


def reviewed_plan(original, edits):
    """Only human review fields can change; keep source identity and annotation counts."""
    result=copy.deepcopy(original)
    expected=[r['file_name'] for r in result['images']]
    if not isinstance(edits,list) or [r.get('file_name') for r in edits]!=expected:
        raise ValueError('Image list or order changed; reload the original plan')
    groups={};counts={'train':0,'val':0,'test':0}
    for row,edit in zip(result['images'],edits):
        for field in ('reviewed','verified_empty'):
            if type(edit.get(field)) is not bool:raise ValueError('Review flags must be booleans')
            row[field]=edit[field]
        for field in ('arrangement_group','split'):
            if not isinstance(edit.get(field),str):raise ValueError('Group/split must be text')
            row[field]=edit[field].strip()
        if not row['reviewed']:continue
        group,split=row['arrangement_group'],row['split']
        if not group or group=='REVIEW_AND_ASSIGN' or split not in counts:
            raise ValueError('Assign group and split for '+row['file_name'])
        if group in groups and groups[group]!=split:
            raise ValueError('Same arrangement appears in multiple splits: '+group)
        groups[group]=split;counts[split]+=1
        if (row['exported_instances']==0)!=row['verified_empty']:
            raise ValueError('Confirm empty scenes explicitly; nonempty scenes cannot be empty: '+row['file_name'])
    if counts['train']<2 or counts['val']<1:
        raise ValueError('Select at least two train images and one val image from independent groups')
    result['target_split']={split:count for split,count in counts.items() if count}
    validate_frozen_reviews(result)
    return result


PAGE=r'''<!doctype html><meta charset="utf-8"><title>YOLO review plan</title>
<style>
body{font:16px system-ui;margin:20px;background:#f4f6f8;color:#17232d}
header{position:sticky;top:0;background:#fff;padding:16px;border:1px solid #ccd;z-index:2}
button,input,select{font:inherit;padding:7px;margin:4px}button{cursor:pointer}
#grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(290px,1fr));gap:12px;margin-top:16px}
article{background:white;padding:10px;border:3px solid transparent;border-radius:8px}
article.selected{border-color:#1668dc}article img{width:100%;cursor:pointer}
small{display:block;overflow-wrap:anywhere}#message{white-space:pre-wrap;color:#175b2e}
</style>
<header>
<b>YOLO review plan — local only</b>
<small>Classes: VARa = large white · VARb = small white · VARc = small red · hand = hand</small>
<p>Select images from ONE physical arrangement. Click the group button to fill all their fields together.
Different hand/robot poses or lighting alone do not make a new arrangement.</p>
<button id="all">Select visible</button><button id="none">Clear selection</button>
<label>Show <select id="filter"><option value="new">New images only</option><option value="all">All images</option><option value="pending">Not reviewed</option><option value="annotated">With annotations</option></select></label>
<input id="group" placeholder="e.g. oct27_layout_01" list="groups"><datalist id="groups"></datalist>
<select id="split"><option value="train">train</option><option value="val">val</option><option value="test">test</option></select>
<label><input type="checkbox" id="empty">I checked selected blank scenes contain no target boards/hands</label>
<br><button id="assign">Confirm selected reviewed + assign group/split</button>
<button id="exclude">Exclude selected from training</button><button id="save">Save new plan</button>
<div id="summary"></div><div id="message"></div>
<small>Thumbnail outlines show exported annotations, not a quality guarantee. Click an image to enlarge it.
No review flag is inferred from a mask count. Saving updates target_split to the actual selected counts.</small>
</header><div id="grid"></div>
<script>
let rows=[],selected=new Set();
const el=id=>document.getElementById(id);
function visible(r){return (el('filter').value==='new'&&!r.locked)||el('filter').value==='all'||(el('filter').value==='pending'&&!r.reviewed&&!r.locked)||(el('filter').value==='annotated'&&r.exported_instances>0)}
function render(){
 el('grid').replaceChildren();
 rows.forEach((r,i)=>{if(!visible(r))return;
  const card=document.createElement('article');card.className=selected.has(i)?'selected':'';
  const box=document.createElement('input');box.type='checkbox';box.checked=selected.has(i);
  box.disabled=!!r.locked;
  box.onchange=()=>{box.checked?selected.add(i):selected.delete(i);render()};
  const name=document.createElement('b');name.textContent=r.file_name;
  const img=document.createElement('img');img.loading='lazy';img.src='/image/'+i;img.alt=r.file_name;
  img.onclick=()=>window.open('/image/'+i+'?full=1','_blank','noopener');
  const info=document.createElement('small');info.textContent=`${r.locked?'LOCKED PREVIOUS IMAGE | ':''}${r.exported_instances} exported rows | ${r.reviewed?'reviewed':'excluded/pending'} | ${r.arrangement_group||'no group'} | ${r.split||'no split'}`;
  card.append(box,name,img,info);el('grid').append(card);
 });
 const count=s=>rows.filter(r=>r.reviewed&&r.split===s).length;
 el('summary').textContent=`${rows.length} images · locked old ${rows.filter(r=>r.locked).length} · new ${rows.filter(r=>!r.locked).length} · selected ${selected.size} · train ${count('train')} · val ${count('val')} · test ${count('test')} · excluded/pending ${rows.filter(r=>!r.reviewed).length}`;
 el('groups').replaceChildren();[...new Set(rows.map(r=>r.arrangement_group).filter(Boolean))].forEach(g=>{const o=document.createElement('option');o.value=g;el('groups').append(o)});
}
el('all').onclick=()=>{rows.forEach((r,i)=>{if(visible(r)&&!r.locked)selected.add(i)});render()};
el('none').onclick=()=>{selected.clear();render()};el('filter').onchange=()=>{selected.clear();render()};
el('group').onchange=()=>{const r=rows.find(r=>r.reviewed&&r.arrangement_group===el('group').value);if(r)el('split').value=r.split};
el('assign').onclick=()=>{
 const group=el('group').value.trim(),split=el('split').value;
 if(!selected.size||!group){el('message').textContent='Select images and enter one arrangement group.';return}
 if(rows.some((r,i)=>!selected.has(i)&&r.reviewed&&r.arrangement_group===group&&r.split!==split)){el('message').textContent='Select every reviewed image in this group to change its split together.';return}
 if([...selected].some(i=>rows[i].exported_instances===0)&&!el('empty').checked){el('message').textContent='Blank scenes require the explicit empty-scene checkbox. Exclude incompletely annotated scenes.';return}
 selected.forEach(i=>Object.assign(rows[i],{reviewed:true,arrangement_group:group,split,verified_empty:rows[i].exported_instances===0}));
 selected.clear();el('empty').checked=false;el('message').textContent='Applied. Save the new plan when finished.';render();
};
el('exclude').onclick=()=>{selected.forEach(i=>{rows[i].reviewed=false;rows[i].verified_empty=false});selected.clear();render()};
el('save').onclick=async()=>{try{
 const response=await fetch('/save/TOKEN',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(rows)});
 const result=await response.json();if(!response.ok)throw Error(result.error);
 el('message').textContent='Saved: '+result.output+'\n'+JSON.stringify(result.target_split);el('save').disabled=true;
 }catch(e){el('message').textContent=e.message}};
fetch('/plan').then(r=>r.json()).then(p=>{rows=p.images;render()}).catch(e=>{el('message').textContent=e.message});
</script>'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--zip',type=Path,required=True)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--output-plan',type=Path,required=True)
    a=p.parse_args()
    if a.output_plan.exists():p.error('Output exists; choose a new output-plan filename')
    plan,images,labels,names=load_plan(a.zip,a.plan)
    token=secrets.token_urlsafe(32)
    @lru_cache(maxsize=64)
    def preview(index,full):
        from PIL import Image, ImageDraw
        row=plan['images'][index];name=row['file_name']
        with Image.open(io.BytesIO(images[name])) as original:image=original.convert('RGB')
        draw=ImageDraw.Draw(image);colours=['cyan','orange','red','magenta']
        for line in labels.get(Path(name).stem,'').splitlines():
            values=line.split()
            if not values:continue
            points=list(zip(map(float,values[1::2]),map(float,values[2::2])))
            xy=[(x*image.width,y*image.height) for x,y in points]
            draw.line(xy+[xy[0]],fill=colours[int(values[0])],width=3)
        image.thumbnail((1920,1080) if full else (480,300))
        out=io.BytesIO();image.save(out,format='JPEG',quality=85);return out.getvalue()
    class Handler(BaseHTTPRequestHandler):
        def reply(self,status,body,kind='application/json'):
            data=body if isinstance(body,bytes) else body.encode()
            self.send_response(status);self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store')
            self.end_headers();self.wfile.write(data)
        def do_GET(self):
            parsed=urlparse(self.path)
            if parsed.path=='/':self.reply(200,PAGE.replace('TOKEN',token),'text/html; charset=utf-8')
            elif parsed.path=='/plan':self.reply(200,json.dumps(plan))
            elif parsed.path.startswith('/image/'):
                try:self.reply(200,preview(int(parsed.path.split('/')[-1]),parsed.query=='full=1'),'image/jpeg')
                except (ValueError,IndexError):self.reply(404,'{}')
            else:self.reply(404,'{}')
        def do_POST(self):
            if self.path!='/save/'+token:self.reply(403,'{}');return
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=2_000_000:raise ValueError('Invalid request size')
                updated=reviewed_plan(plan,json.loads(self.rfile.read(size)))
                a.output_plan.parent.mkdir(parents=True,exist_ok=True)
                with a.output_plan.open('x') as out:json.dump(updated,out,indent=2)
                self.reply(200,json.dumps({'output':str(a.output_plan.resolve()),'target_split':updated['target_split']}))
                print('Saved:',a.output_plan.resolve(),flush=True)
            except (ValueError,KeyError,TypeError,OSError) as error:
                self.reply(400,json.dumps({'error':str(error)}))
        def log_message(self,*args):pass
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    print(f'Open in your browser: http://127.0.0.1:{server.server_port}',flush=True)
    print('Keep this terminal running. Save the plan in the browser, then Ctrl+C here.',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()


if __name__=='__main__':main()
