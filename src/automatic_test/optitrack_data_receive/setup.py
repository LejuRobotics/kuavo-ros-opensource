from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 自动生成setup所需的参数
d = generate_distutils_setup(
    packages=['optitrack_data_receive'],  # 包名
    package_dir={'': 'scripts'},  # 包源目录
)

setup(**d)