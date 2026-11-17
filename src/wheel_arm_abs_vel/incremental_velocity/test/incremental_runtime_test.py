#!/usr/bin/env python3
"""Exercise the production shell dispatch, including resolved underlay paths."""
import json,os,subprocess,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
WRAPPER=ROOT/'scripts/whole_body_ik_runtime.sh'
class RuntimeDispatchTest(unittest.TestCase):
 def test_private_helpers_precede_common_runtime_pythonpath(self):
  package=ROOT
  private=package/'incremental_velocity/scripts'
  env=dict(os.environ,PYTHONPATH=str(private)+':'+str(package/'scripts')+':'+str(package/'scripts/tools')+':'+os.environ.get('PYTHONPATH',''))
  result=subprocess.run(['/usr/bin/python3','-c','import ik_ros_velocity, pose_velocity_kalman; print(pose_velocity_kalman.__file__)'],env=env,capture_output=True,text=True,timeout=30)
  self.assertEqual(result.returncode,0,result.stderr)
  self.assertEqual(Path(result.stdout.strip()),private/'tools/pose_velocity_kalman.py')
 def test_selected_program_survives_launch_prefix(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp)
   (root/'rospack').write_text('#!/bin/sh\nprintf "%s\\n" "'+str(ROOT)+'"\n')
   (root/'rospack').chmod(0o755)
   env=dict(os.environ,PATH=str(root)+':'+os.environ['PATH'],KUAVO_RUNTIME_PARTITION='0')
   for role,name in [('ik','pinocchio_velocity_ik_node'),('ik','pinocchio_incremental_velocity_ik_node'),('mapper','ik_ros_velocity.py'),('mapper','run_incremental_velocity.py')]:
    path=root/name
    path.write_text('#!/usr/bin/python3\nimport sys,json\nprint(json.dumps(sys.argv))\n')
    path.chmod(0o755)
    result=subprocess.run(['bash',str(WRAPPER),role,str(path),'__name:=test'],env=env,capture_output=True,text=True,check=True)
    self.assertEqual(json.loads(result.stdout),[str(path),'__name:=test'])
if __name__=='__main__':unittest.main()
