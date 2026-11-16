"""Build locally; requires pybind11 and Eigen 3.4 headers. No deployment."""
import argparse,os,subprocess,sysconfig
from pathlib import Path
import pybind11
p=argparse.ArgumentParser();p.add_argument('--eigen',required=True,type=Path);a=p.parse_args()
here=Path(__file__).resolve().parent;out=here.parent/('_mapper_numeric_hands'+sysconfig.get_config_var('EXT_SUFFIX'))
cmd=[os.environ.get('CXX','c++'),'-O3','-std=c++17','-shared','-fPIC','-ffp-contract=off','-fno-fast-math','-DEIGEN_DONT_PARALLELIZE','-I'+str(a.eigen.resolve()),'-I'+pybind11.get_include(),'-I'+sysconfig.get_paths()['include'],str(here/'mapper_numeric.cpp'),'-o',str(out)]
if sysconfig.get_platform().startswith('macosx'):cmd+=['-undefined','dynamic_lookup']
subprocess.run(cmd,check=True);print(out)
