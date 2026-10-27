"""Flange-to-cup conversion and downstream verification with synthetic references."""
import csv
from contextlib import redirect_stdout
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1]


def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/f'{name}.py')
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


imp=module('import_pendant_points')


def csv_text(angles=(180,0,0),**overrides):
    stream=io.StringIO()
    fields=['label','x_mm','y_mm','z_mm','rx_deg','ry_deg','rz_deg','tool_slot','user_frame']
    writer=csv.DictWriter(stream,fieldnames=fields)
    writer.writeheader()
    for label in imp.REQUIRED:
        row=dict(zip(fields,[label,400,-200,150,*angles,'','']))
        row.update(overrides)
        writer.writerow(row)
    return stream.getvalue()


class FlangeImportTests(unittest.TestCase):
    def convert(self,text=None,**kwargs):
        return imp.convert(text or csv_text(),'tcp',input_frame='flange',
                           offset_m=[0,0,.1129],**kwargs)

    def test_actual_urdf_offset_and_provenance(self):
        offset,sources=imp.model_tool_offset()
        self.assertAlmostEqual(offset[2],.1129)
        self.assertEqual(offset[:2],[0,0])
        self.assertEqual(len(sources),2)
        for item in sources:
            self.assertEqual(item['sha256'],hashlib.sha256(Path(item['path']).read_bytes()).hexdigest())
        with tempfile.TemporaryDirectory() as tmp:
            changed=Path(tmp)/'gripper.xacro'
            original=(imp.URDF/'schmalz_gripper.xacro').read_text()
            changed.write_text(original.replace('yaw:=0.0','yaw:=0.2'))
            with self.assertRaisesRegex(ValueError,'Rotated tool chain'):
                imp.model_tool_offset(gripper_path=changed)

    def test_downward_flange_conversion_preserves_raw_data(self):
        row=self.convert()[0]
        for got,expected in zip(row['position_m'],[-.4,.2,.0371]):
            self.assertAlmostEqual(got,expected)
        self.assertEqual(row['pendant_position_mm'],[400,-200,150])
        self.assertIsNone(row['pendant_tool_slot'])
        self.assertFalse(row['physical_tip_calibration_verified'])
        self.assertFalse(row['contact_confirmed'])
        self.assertFalse(row['execution_allowed'])

    def test_tilt_and_yaw_change_xy_not_just_z(self):
        row=self.convert(csv_text(angles=(0,90,90)))[0]
        for got,expected in zip(row['position_m'],[-.4,.0871,.15]):
            self.assertAlmostEqual(got,expected)
        # Three quarter-turns give (1,2,3) -> (1,-3,2) -> (2,-3,-1) -> (3,2,-1).
        for got,want in zip(imp.rotate_rpy([1,2,3],[90,90,90]),[3,2,-1]):
            self.assertAlmostEqual(got,want)

    def test_contact_and_geometry_confirmations_are_separate(self):
        row=self.convert(contact_confirmed=True)[0]
        self.assertTrue(row['contact_confirmed'])
        self.assertFalse(row['physical_tip_calibration_verified'])
        row=self.convert(geometry_verified=True)[0]
        self.assertFalse(row['contact_confirmed'])
        self.assertTrue(row['physical_tip_calibration_verified'])

    def test_bad_orientation_frame_and_csv_rejected(self):
        for changes in ({'rx_deg':''},{'ry_deg':'nan'},{'rz_deg':361},{'user_frame':'2'}):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.convert(csv_text(**changes))
        with self.assertRaisesRegex(ValueError,'column count'):
            self.convert(csv_text().replace('180,0,0','180,7,576,0',1))
        with self.assertRaises(ValueError):
            imp.convert(csv_text(),'tcp',input_frame='flange')

    def test_tcp_input_still_requires_tool_slot(self):
        with self.assertRaisesRegex(ValueError,'tool_slot'):
            imp.convert(csv_text(user_frame='0'),'tcp')
        row=imp.convert(csv_text(tool_slot='1',user_frame='0'),'tcp',True)[0]
        self.assertEqual(row['position_m'],[-.4,.2,.15])

    def test_cli_does_not_overwrite_previous_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'points.csv';source.write_text(csv_text())
            target=Path(tmp)/'points.jsonl'
            command=[sys.executable,str(ROOT/'scripts/import_pendant_points.py'),
                     '--csv',str(source),'--output',str(target),
                     '--input-frame','flange','--confirm-pendant-base']
            first=subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(first.returncode,0,first.stderr)
            saved=target.read_bytes()
            self.assertIn('tool_geometry_sources',json.loads(saved.splitlines()[0]))
            second=subprocess.run(command,capture_output=True,text=True)
            self.assertNotEqual(second.returncode,0)
            self.assertEqual(target.read_bytes(),saved)


class HomographyVerificationTests(unittest.TestCase):
    def test_candidate_references_cannot_be_promoted_by_solve_flag(self):
        # Exercise both homography entry points, including the excluded rail labels.
        for name in ('homography_no_rail','homography_session'):
            h=module(name)
            for verified,warped in ((False,False),(True,False),(True,True)):
                with self.subTest(script=name,verified=verified,warped=warped),tempfile.TemporaryDirectory() as tmp:
                    root=Path(tmp)
                    xyz={
                        'F1':[0,0,.1],'F2':[.4,0,.1],'F3':[.4,.3,.1],
                        'F4':[0,.3,.1],'F5':[.2,.15,.1],
                        'V1':[.1,.1,.1],'V2':[.3,.1,.1],
                        'V3':[.1,.2,.1],'V4':[.3,.2,.1],
                    }
                    if warped:
                        for label,delta in zip(('F1','F2','F3','F4'),(.02,-.02,.02,-.02)):
                            xyz[label][2]+=delta
                    labels=imp.REQUIRED if name=='homography_no_rail' else h.LABELS
                    points=root/'points.jsonl'
                    points.write_text(''.join(json.dumps({
                        'label':v,'position_m':xyz[v],'frame_id':'base_link','tip_frame':'tcp',
                        'input_pose_frame':'flange','contact_confirmed':verified,
                        'physical_tip_calibration_verified':verified,
                    })+'\n' for v in labels))
                    image=root/'reference.png';image.write_bytes(b'unchanged reference hash')
                    (root/'pixels.json').write_text(json.dumps({
                        'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
                        'pixels':{v:[xyz[v][0]*1000+100,xyz[v][1]*1000+200] for v in labels},
                    }))
                    intrinsics=root/'intrinsics.json'
                    intrinsics.write_text(json.dumps({
                        'distortion_model':'plumb_bob','image_size':[640,480],
                        'camera_matrix':[[800,0,320],[0,800,240],[0,0,1]],
                        'distortion_coefficients':[0]*5,
                    }))
                    args=SimpleNamespace(points=points,image=image,folder=root,tip='tcp',
                        intrinsics=intrinsics,limit_mm=5,exclude=['F5','V3'],
                        reference_contact_verified=True)
                    with redirect_stdout(io.StringIO()):h.solve(args)
                    result=json.loads((root/'homography_validation.json').read_text())
                    self.assertEqual(result['independent_check_passed'],not warped)
                    self.assertEqual(result['reference_contact_and_tcp_verified_by_operator'],verified)
                    self.assertFalse(result['execution_allowed'])


if __name__=='__main__':unittest.main()
