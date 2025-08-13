# 如何使用 KuavoPoseCalculator 获取相机外参

## 依赖

这个类使用到了 drake，需要安装对应的包，先执行命令看看有没有已安装:
```bash
dpkg -l |grep drake
ii  drake-dev                                     1.19.0-1                             amd64        Model-based design/verification for robotics
```
没有的话则按照如下命令安装:
```bash
apt-get update -y && apt-get install --no-install-recommends ca-certificates gnupg lsb-release wget -y \
    && wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null \
    && echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/drake.list >/dev/null \
    && apt-get update -y \
    && apt-get install --no-install-recommends drake-dev=1.19.0-1 -y 
```

安装完之后需要添加环境变量:

```bash
# 添加用户环境变量
echo 'export PATH="/opt/drake/bin${PATH:+:${PATH}}"' >> ~/.bashrc
echo "export PYTHONPATH=\"/opt/drake/lib/python\$(python3 -c 'import sys; print(\"{0}.{1}\".format(*sys.version_info))')/site-packages\${PYTHONPATH:+:\${PYTHONPATH}}\"" >> ~/.bashrc

# 添加 root 环境变量
sudo su
echo 'export PATH="/opt/drake/bin${PATH:+:${PATH}}"' >> /root/.bashrc
echo "export PYTHONPATH=\"/opt/drake/lib/python\$(python3 -c 'import sys; print(\"{0}.{1}\".format(*sys.version_info))')/site-packages\${PYTHONPATH:+:\${PYTHONPATH}}\"" >> /root/.bashrc
```

## 使用

`KuavoPoseCalculator` 目前提供了 3 个函数用于获取头部，左右手相机的外参:
```python
# 需要使用您实际的urdf文件路径
kuavo_camera_pose_calculator = KuavoPoseCalculator("../../src/kuavo_assets/models/biped_s45/urdf/biped_s45.urdf")

def get_camera_pose(self, head_q:list)
def get_l_hand_camera_pose(self, rarm_q:list):
def get_r_hand_camera_pose(self, rarm_q:list):
```

**使用示例:** example_camera_pose_from_bag.py