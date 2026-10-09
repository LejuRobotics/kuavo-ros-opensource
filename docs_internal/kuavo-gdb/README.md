# Kuavo-GDB

## 描述
Kuavo-GDB 是一个用于分析 Kuavo 机器人崩溃日志的调试工具。该工具可以帮助开发人员快速定位崩溃原因，查看崩溃时的调用栈和变量状态，从而更高效地解决问题。

该工具需要在 Kuavo 开发环境中运行，并依赖于一些调试相关的软件包，如 GDB、ROS 调试符号等。

工具会通过文件头 magic byte 自动识别 `.tar.gz` 和 `.tar.zst` 两种格式的归档，无需手动指定。

使用方法如下：
> ```bash
> ./docs_internal/kuavo-gdb/kuavo-gdb.sh 
> Usage: ./docs_internal/kuavo-gdb/kuavo-gdb.sh [options]
> Options:
>   -c, --crash_zip <kuavo-crash filename>  Specify the crash zip file to analyze
>   -d, --download-debug-symbol <kuavo-dbgsym filename> Download debug symbols
>   -h, --help                             Display this help message
> 
> Example:
>   ./docs_internal/kuavo-gdb/kuavo-gdb.sh -c kuavo-crash_2025-04-09-15-48_4cfe4790a55bb686c854f1cdce71ed9e38116ef4.tar.gz
>   ./docs_internal/kuavo-gdb/kuavo-gdb.sh -c kuavo-crash_2025-04-09-15-48_4cfe4790a55bb686c854f1cdce71ed9e38116ef4.tar.zst
>   ./docs_internal/kuavo-gdb/kuavo-gdb.sh -d kuavo-dbgsym_4cfe4790a55bb686c854f1cdce71ed9e38116ef4.tar.gz
> ```

用户上传`kuavo-crash`后会提供如下的信息给开发支持人员，比如:
```bash
用户日志已上传, 详情请查看: kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4.tar.gz
```

开发支持人员需要将`kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4.tar.gz`作为参数，调用本工具:
```bash
chmod +x ./docs_internal/kuavo-gdb/kuavo-gdb.sh
sudo su
./docs_internal/kuavo-gdb/kuavo-gdb.sh -c kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4.tar.gz

# 以下是示例输出：
GIT_COMMIT: 7eea36d8fbe9814f9add644b2accc6f0b600768d
Core dump files already exist in /home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps
Debug symbols already exist for commit 7eea36d8fbe9814f9add644b2accc6f0b600768d
Core dump files already exist in /home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps
Using existing crash files
Debug symbols already exist for commit 7eea36d8fbe9814f9add644b2accc6f0b600768d
Reading crash information from /home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/info.txt
-----------------------------------------------
Crash: 
 -branch: deve
 -crash commit: b7f0cacc957b258696040e6b11be2b8b77b9b15a
 -sync commit: 7eea36d8fbe9814f9add644b2accc6f0b600768d
 -repo: https://gitee.com/leju-robot/kuavo-ros-opensource.git
Local:
 -branch: HEAD
 -commit: 7eea36d8fbe9814f9add644b2accc6f0b600768d
 -repo: https://www.lejuhub.com/highlydynamic/kuavo-ros-control.git
-----------------------------------------------
Commit match: Yes
Multiple coredump files found. Please select one:
[1] core.humanoid_quest_.534579.1744976596
[2] core.nodelet.534604.1744976595
[3] core.humanoid_VR_han.534568.1744976596
[4] core.humanoid_hand_c.534553.1744976596
[5] core.humanoid_sqp_mp.534541.1744976596
Enter selection number [1-5]: 2
Selected coredump file: core.nodelet.534604.1744976595
- Executable file: ../../../../opt/ros/noetic/lib/nodelet/nodelet
- Debug symbols directory: kuavo-gdb-debug/symbols/7eea36d8fbe9814f9add644b2accc6f0b600768d/debug
- Core file directory: kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps
- Source repository path: /media/data/gitlab-runner/builds/8rHoW4Dt/0/highlydynamic/kuavo-ros-control
- Source project path: /home/kuavo/kuavo-ros-control
--------------------------------------------------------------------------------
gdb
-ex "set debug-file-directory /home/kuavo/kuavo-ros-control/kuavo-gdb-debug/symbols/7eea36d8fbe9814f9add644b2accc6f0b600768d/debug:/usr/lib/debug"                                                                                                                                              
-ex "set solib-search-path /home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps/../installed/lib:/home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps/../devel/lib"                                                                                                                                                                                                                                                                                                                         
-ex "set substitute-path /media/data/gitlab-runner/builds/8rHoW4Dt/0/highlydynamic/kuavo-ros-control /home/kuavo/kuavo-ros-control"                                                                                                                               
-ex "file /opt/ros/noetic/lib/nodelet/nodelet"                                        
-ex "core-file /home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps/core.nodelet.534604.1744976595"                                                                                                                                                                           
--------------------------------------------------------------------------------
GNU gdb (Ubuntu 9.2-0ubuntu1~20.04.2) 9.2
...
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word".
Reading symbols from /opt/ros/noetic/lib/nodelet/nodelet...
Reading symbols from /usr/lib/debug/.build-id/e0/356d7780ebc39e92381cd0d949e528423d2101.debug...
[New LWP 537166]
[New LWP 535097]
...
[New LWP 536445]
warning: .dynamic section for "/opt/ros/noetic/lib/x86_64-linux-gnu/libhpp-fcl.so" is not at the expected address (wrong library or version mismatch?)
warning: .dynamic section for "/usr/lib/python3.8/lib-dynload/_ssl.cpython-38-x86_64-linux-gnu.so" is not at the expected address (wrong library or version mismatch?)
warning: Could not load shared library symbols for 33 libraries, e.g. /opt/ros/noetic/lib/liboctomap.so.1.9.
Use the "info sharedlibrary" command to see the complete listing.
Do you need "set solib-search-path" or "set sysroot"?
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".
--Type <RET> for more, q to quit, c to continue without paging--
Core was generated by `/opt/ros/noetic/lib/nodelet/nodelet manager __name:=nodelet_manager __log:=/roo'.
Program terminated with signal SIGSEGV, Segmentation fault.
#0  HighlyDynamic::HardwareNode::<lambda()>::operator() (__closure=<optimized out>)
    at /media/data/gitlab-runner/builds/8rHoW4Dt/0/highlydynamic/kuavo-ros-control/src/kuavo-ros-control-lejulib/hardware_node/src/hardware_plant.cc:1119
warning: Source file is more recent than executable.
1119                *null_ptr = 42;
[Current thread is 1 (Thread 0x7f5bd77f2700 (LWP 537166))]
(gdb) 
```

## Kuavo-Crash 格式说明

#### 压缩包命名规则

**格式为：`kuavo-crash_${TIMESTAMP}_${GIT_COMMIT}.tar.gz` 或 `.tar.zst`，其中：**
- `TIMESTAMP`: 时间，如2025-04-11_15-03-00
- `GIT_COMMIT`：git 提交id，如4cfe4790a55bb686c854f1cdce71ed9e38116ef4

例子：`kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4.tar.gz`

#### 压缩包内容

压缩包中包含stdout日志，coredump文件，以及roslaunch的信息：
- `coredumps`目录包含coredump文件
- `stdout`目录包含log日志
- `info.txt`描述这次roslaunch的信息，包含git的信息，roslaunch_id和机器人名称等

```bash
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/coredumps/
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/coredumps/core.humanoid_quest_.478868.1744355024
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/coredumps/core.humanoid_sqp_mp.478837.1744355024
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/coredumps/core.humanoid_VR_han.478861.1744355024
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/coredumps/core.humanoid_hand_c.478851.1744355024
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/coredumps/core.nodelet.478900.1744355024
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/info.txt
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/stdout/
kuavo-crash_2025-04-11_15-03-00_4cfe4790a55bb686c854f1cdce71ed9e38116ef4/stdout/stdout.log
```

其中`info.txt`的内容类似如下:
```bash
launch_id: 478744
date: 2025-04-11_15-03-00
remote: https://gitee.com/leju-robot/kuavo-ros-opensource.git
branch: master            # 发生crash的分支，用于定位crash
crash_commit: 4cfe4790a55bb686c854f1cdce71ed9e38116ef4      # 发生crash的commit hash，用于定位crash
sync_commit: 15638707f78ecdd84bf198810e7efd5563e4f133 # 同步闭源仓库的commit hash，符号信息通过该commit获取
ROBOT_NAME: kuavo_x.x
coredump_dir: /root/.ros/coredumps/478744
```

## 细节

| 命令                          | 参数/路径                                                                                                                                                                                                                | 用途解释                                                                 |
|------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------|
| `set debug-file-directory`   | `/home/kuavo/kuavo-ros-control/kuavo-gdb-debug/symbols/7eea36d8fbe9814f9add644b2accc6f0b600768d/debug:/usr/lib/debug`                                                                                                    | **设置调试符号搜索路径**<br>从以下目录查找调试符号文件：<br>1. 自定义路径；<br>2. 系统调试符号目录 `/usr/lib/debug`。 |
| `set solib-search-path`      | `/home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps/../installed/lib:/home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps/../devel/lib` | **设置共享库搜索路径**<br>从以下目录查找动态链接库：<br>1. `installed/lib`；<br>2. `devel/lib`。 |
| `set substitute-path`        | 原路径：`/media/data/gitlab-runner/builds/8rHoW4Dt/0/highlydynamic/kuavo-ros-control`<br>替换路径：`/home/kuavo/kuavo-ros-control`                                                                                       | **路径替换规则**<br>将编译时路径替换为当前系统的实际路径，确保 GDB 能找到源代码。 |
| `file`                       | `/opt/ros/noetic/lib/nodelet/nodelet`                                                                                                                                                                                    | **加载可执行文件**<br>指定要调试的可执行程序路径（ROS 的 nodelet 节点）。 |
| `core-file`                  | `/home/kuavo/kuavo-ros-control/kuavo-gdb-debug/kuavo-crash_2025-04-18_19-41-53_7eea36d8fbe9814f9add644b2accc6f0b600768d/coredumps/core.nodelet.534604.1744976595`                                                         | **加载核心转储文件**<br>指定程序崩溃时的核心文件，用于分析崩溃原因。 |
