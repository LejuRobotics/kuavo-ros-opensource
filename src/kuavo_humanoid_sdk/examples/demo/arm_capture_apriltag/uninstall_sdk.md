```bash
root@NUC11TNKi7:/home/lab/kuavo-ros-control# pip uninstall kuavo-humanoid-sdk-ws 
Found existing installation: kuavo-humanoid-sdk-ws 1.2.1a1772 Can't uninstall 'kuavo-humanoid-sdk-ws'. No files were found to uninstall.
```
这个提示说明：  
`pip` 确认有 `kuavo-humanoid-sdk-ws` 的元数据（记录在 `site-packages` 的 `.egg-info` 或 `.dist-info`），但实际的文件夹已经不存在，所以 `pip` 找不到可卸载的文件。  

你这里安装方式是源码路径安装（Location 指向 `/home/lab/kuavo-ros-control/src/kuavo_humanoid_websocket_sdk`），而不是复制到 `site-packages` 里，所以删除时 pip 不知道怎么卸载。

---

### 解决方法：

#### 1. 手动删除源码文件夹
```bash
rm -rf /home/lab/kuavo-ros-control/src/kuavo_humanoid_websocket_sdk
```

#### 2. 手动删除 pip 的元数据
找到对应的 `.egg-info` 或 `.dist-info` 目录，一般在：
```bash
ls -d /usr/local/lib/python*/dist-packages/kuavo_humanoid_sdk_ws* /usr/lib/python*/dist-packages/kuavo_humanoid_sdk_ws* 2>/dev/null
```
或者：
```bash
pip show -f kuavo-humanoid-sdk-ws
```
这会显示出 pip 记录的文件路径。  
找到后，删除它：
```bash
rm -rf /usr/local/lib/python3.*/dist-packages/kuavo_humanoid_sdk_ws-*.egg-info
rm -rf /usr/local/lib/python3.*/dist-packages/kuavo_humanoid_sdk_ws-*.dist-info
```

#### 3. 确认是否删除干净
```bash
pip show kuavo-humanoid-sdk-ws
```
如果没有输出，说明彻底清理了。

---
