"""Import pendant-base TCP or flange poses as cup-tip positions. No motion."""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

REQUIRED=['F1','F2','F3','F4','V1','V2','V4']
ROOT=Path(__file__).resolve().parents[1]
URDF=ROOT/'src/fr5_bringup/urdf'
XACRO='{http://www.ros.org/wiki/xacro}'


def model_tool_offset(assembly_path=URDF/'fr5_workcell.urdf.xacro',
                      gripper_path=URDF/'schmalz_gripper.xacro'):
    """Read this project's literal, unrotated flange-to-cup chain; fail on changes."""
    assembly=ET.parse(assembly_path).getroot()
    gripper=ET.parse(gripper_path).getroot()
    adapter=assembly.find("joint[@name='tool_adapter_joint']")
    macro=gripper.find(XACRO+"macro[@name='schmalz_gripper']")
    calls=assembly.findall(XACRO+'schmalz_gripper')
    if adapter is None or macro is None or len(calls)!=1:
        raise ValueError('Cannot identify the URDF flange-to-cup chain')
    def check_joint(joint,parent,child,xyz):
        if (joint is None or joint.get('type')!='fixed'
                or any(joint.find(tag) is None for tag in ('parent','child','origin'))
                or joint.find('parent').get('link')!=parent
                or joint.find('child').get('link')!=child
                or joint.find('origin').get('xyz')!=xyz):
            raise ValueError('URDF tool chain changed; review flange-to-TCP conversion')
    origin=adapter.find('origin')
    if origin is None:raise ValueError('URDF adapter origin is missing')
    check_joint(adapter,'flange','tool_adapter',origin.get('xyz'))
    check_joint(macro.find("joint[@name='gripper_mount']"),'${parent}','gripper_link','0 0 0')
    tcp=macro.find("joint[@name='tcp_joint']")
    check_joint(tcp,'gripper_link','tcp','0 0 ${tcp_z}')
    defaults=dict(v.split(':=',1) for v in macro.get('params','').split() if ':=' in v)
    if (adapter.find('origin').get('rpy','0 0 0')!='0 0 0'
            or macro.find("joint[@name='gripper_mount']/origin").get('rpy')!='0 0 ${yaw}'
            or tcp.find('origin').get('rpy','0 0 0')!='0 0 0'
            or calls[0].get('parent')!='tool_adapter'
            or float(calls[0].get('yaw',defaults['yaw']))!=0):
        raise ValueError('Rotated tool chain requires reviewing the conversion')
    tcp_z=float(calls[0].get('tcp_z',defaults['tcp_z']))
    offset=[float(v) for v in adapter.find('origin').get('xyz').split()]
    if len(offset)!=3 or not all(math.isfinite(v) for v in offset+[tcp_z]) or tcp_z<=0:
        raise ValueError('Invalid URDF tool offset')
    offset[2]+=tcp_z
    sources=[{'path':str(p.resolve()),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
             for p in (assembly_path,gripper_path)]
    return offset,sources


def rotate_rpy(vector,angles_deg):
    # FAIRINO fixed-axis X/Y/Z angles in degrees: R = Rz(rz) Ry(ry) Rx(rx).
    rx,ry,rz=map(math.radians,angles_deg)
    sx,cx=math.sin(rx),math.cos(rx)
    sy,cy=math.sin(ry),math.cos(ry)
    sz,cz=math.sin(rz),math.cos(rz)
    x,y,z=vector
    y,z=cx*y-sx*z,sx*y+cx*z
    x,z=cy*x+sy*z,-sy*x+cy*z
    return [cz*x-sz*y,sz*x+cz*y,z]


def convert(text,tip,confirmed=False,*,input_frame='tcp',offset_m=None,
            contact_confirmed=False,geometry_verified=False):
    if input_frame not in ('tcp','flange'):raise ValueError('Unknown input frame')
    if input_frame=='flange':
        if tip!='tcp':raise ValueError('Flange conversion outputs the model tcp frame')
        if (offset_m is None or len(offset_m)!=3
                or not all(math.isfinite(v) for v in offset_m)
                or not 0<math.sqrt(sum(v*v for v in offset_m))<.5):
            raise ValueError('A finite URDF flange-to-TCP offset is required')
    rows=list(csv.DictReader(io.StringIO(text)))
    labels=[row.get('label','').strip() for row in rows]
    if len(labels)!=len(set(labels)):raise ValueError('Duplicate labels in CSV')
    selected=[row for row,label in zip(rows,labels) if label in REQUIRED]
    if set(row['label'].strip() for row in selected)!=set(REQUIRED):
        raise ValueError('Need F1-F4, V1, V2 and V4; no missing required rows')
    for row in selected:
        if None in row or any(value is None for value in row.values()):
            raise ValueError(f"{row['label']}: wrong CSV column count; use decimal points, not decimal commas")
    slots={row.get('tool_slot','').strip() for row in selected}
    if input_frame=='tcp' and ('' in slots or len(slots)!=1):
        raise ValueError('Record one consistent calibrated tool_slot for all seven points')
    result=[]
    for row in selected:
        label=row['label'].strip()
        frame=row.get('user_frame','').strip().lower()
        # A blank flange frame is resolved only by the CLI's required BASE confirmation.
        if frame not in ('0','base') and not (input_frame=='flange' and frame==''):
            raise ValueError(f'{label}: expected confirmed pendant BASE readout (user_frame 0/base)')
        try:xyz=[float(row[key]) for key in ('x_mm','y_mm','z_mm')]
        except (KeyError,TypeError,ValueError):raise ValueError(f'{label}: fill numeric x_mm,y_mm,z_mm') from None
        if not all(math.isfinite(v) for v in xyz):raise ValueError(f'{label}: non-finite coordinates')
        if max(abs(v) for v in xyz)>2000:raise ValueError(f'{label}: implausible FR5 base millimetres; check units/frame')
        tip_xyz=[v/1000 for v in xyz]
        pose_metadata={}
        if input_frame=='flange':
            try:angles=[float(row[key]) for key in ('rx_deg','ry_deg','rz_deg')]
            except (KeyError,TypeError,ValueError):
                raise ValueError(f'{label}: fill numeric flange rx_deg, ry_deg, rz_deg') from None
            if not all(math.isfinite(v) and abs(v)<=360 for v in angles):
                raise ValueError(f'{label}: invalid flange angles; expected degrees within +/-360')
            delta=rotate_rpy(offset_m,angles)
            tip_xyz=[v+d for v,d in zip(tip_xyz,delta)]
            pose_metadata={'pendant_flange_rpy_deg':angles,
                           'flange_to_tcp_translation_m':list(offset_m),
                           'offset_in_pendant_base_m':delta,
                           'pendant_tip_position_mm':[v*1000 for v in tip_xyz],
                           'rotation_convention':'fixed XYZ degrees; Rz(rz) @ Ry(ry) @ Rx(rx)'}
        result.append({'label':label,'frame_id':'base_link','tip_frame':tip,
                       'position_m':[-tip_xyz[0],-tip_xyz[1],tip_xyz[2]],
                       'input_pose_frame':input_frame,
                       'pendant_position_mm':xyz,'pendant_tool_slot':row.get('tool_slot','').strip() or None,
                       'pendant_user_frame':frame or 'base',
                       'pendant_user_frame_source':'CSV' if frame else '--confirm-pendant-base',
                       'physical_tip_calibration_verified':bool(confirmed or geometry_verified),
                       'contact_confirmed':bool(confirmed or contact_confirmed),'execution_allowed':False,
                       'source':f'Manual pendant {input_frame} BASE readout; project base_link = Rz(pi) * pendant_base',
                       **pose_metadata})
    return sorted(result,key=lambda row:REQUIRED.index(row['label']))


def self_test():
    text='label,x_mm,y_mm,z_mm,tool_slot,user_frame\n'+''.join(f'{v},400,-200,50,1,0\n' for v in REQUIRED)
    rows=convert(text,'tcp',True)
    assert len(rows)==7 and rows[0]['position_m']==[-.4,.2,.05]
    assert all(abs(a-b)<1e-12 for a,b in zip(rotate_rpy([0,0,.1],[180,0,0]),[0,0,-.1]))
    assert all(abs(a-b)<1e-12 for a,b in zip(rotate_rpy([0,0,.1],[0,90,90]),[0,.1,0]))
    for bad in [text.replace('F1,400','F1,nan'),text.replace(',1,0',',1,2'),text+text.splitlines()[1]+'\n',text.replace('F1,400','F1,')]:
        try:convert(bad,'tcp',True)
        except ValueError:pass
        else:raise AssertionError('Invalid CSV accepted')
    print('PASS: mm-to-m, base conversion, fixed-XYZ orientation, invalid coordinate rejection')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--self-test',action='store_true')
    p.add_argument('--csv',type=Path)
    p.add_argument('--output',type=Path)
    p.add_argument('--tip',default='tcp')
    p.add_argument('--input-frame',choices=['tcp','flange'],default='tcp',
                   help='Frame described by the pendant pose; flange needs rx/ry/rz in degrees')
    p.add_argument('--confirm-pendant-base',action='store_true')
    p.add_argument('--confirm-calibrated-tip-contact',action='store_true')
    p.add_argument('--confirm-tip-contact',action='store_true',
                   help='Cup contact centre actually coincided with every recorded mark')
    p.add_argument('--confirm-tool-geometry',action='store_true',
                   help='Physical relaxed-cup offset has been verified against the URDF')
    a=p.parse_args()
    if a.self_test:self_test();return
    if not a.csv or not a.output or not a.confirm_pendant_base:
        p.error('--csv, --output and --confirm-pendant-base are required')
    offset,sources=model_tool_offset() if a.input_frame=='flange' else (None,[])
    rows=convert(a.csv.read_text(),a.tip,a.confirm_calibrated_tip_contact,
                 input_frame=a.input_frame,offset_m=offset,
                 contact_confirmed=a.confirm_tip_contact,geometry_verified=a.confirm_tool_geometry)
    checksum=hashlib.sha256(a.csv.read_bytes()).hexdigest()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as stream:
        for row in rows:
            row['source_csv_sha256']=checksum
            if sources:row['tool_geometry_sources']=sources
            stream.write(json.dumps(row,allow_nan=False)+'\n')
    for row in rows:print(row['label'],row['position_m'],'m in base_link')
    print('Saved:',a.output)
    if offset is not None:print('URDF flange-to-TCP offset (m):',offset)
    print('Contact confirmed:',all(row['contact_confirmed'] for row in rows))
    print('Physical TCP geometry verified:',all(row['physical_tip_calibration_verified'] for row in rows))


if __name__=='__main__':main()
