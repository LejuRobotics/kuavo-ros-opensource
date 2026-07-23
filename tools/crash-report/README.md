# Kuavo-CrashReport

## 描述

Kuavo-CrashReport 是一个用于收集、归档和上传夸父机器人崩溃日志的工具。

**温馨提示: 若您需要分析和帮助，强烈建议您在异常发生后第一时间执行本脚本反馈，上传完成后再继续下一次运行或其他操作。**

## 使用方法

### 首先确认您需要上传的日志目录

目前日志存放在 `~/.ros/stdout/` 目录下，因此您需要确认您需要上传的日志目录:
```bash
ls -lt ~/.ros/stdout/
```

### 上传日志文件

**比如我们选择上传`~/.ros/stdout/2025-04-10_14-30-45/`**

```bash
chmod +x ./tools/crash-report/CrashReport.sh
sudo su
./tools/crash-report/CrashReport.sh ~/.ros/stdout/2025-04-10_14-30-45/

# 以下是示例输出：

********************************************************************************
*                                                                              *
*                        Kuavo Crash Report Tool                               *
*                                                                              *
********************************************************************************

LAUNCH_ID: 478744
TIMESTAMP: 2025-04-11_15-03-00
GIT_BRANCH: chenmingfu/feat/tool/kuavo-crash-report
GIT_COMMIT: f1e881a74
-------------------------
📦 开始归档文件...
📦 正在压缩文件: /tmp/kuavo-crash/kuavo-crash_2025-04-11_15-03-00_f1e881a74.tar.gz
1.14GiB 0:00:07 [ 166MiB/s] [              <=>                                                                            ]
Successfully created archive: /tmp/kuavo-crash/kuavo-crash_2025-04-11_15-03-00_f1e881a74.tar.gz (129M)
📤 正在上传文件: /tmp/kuavo-crash/kuavo-crash_2025-04-11_15-03-00_f1e881a74.tar.gz
#################################################################################################################### 100.0%

🙏 感谢您的配合，📦 日志文件已上传成功，请将如下信息复制给乐聚技术支持人员:

用户日志已上传, 详情请查看: kuavo-crash_2025-04-11_15-03-00_f1e881a74.tar.gz
```

### 请拷贝消息给乐聚支持人员
```bash
用户日志已上传, 详情请查看: kuavo-crash_2025-04-11_15-03-00_f1e881a74.tar.gz
```

## 其他

### 压缩格式

工具默认使用 `zstd -19` 压缩 (产出 `.tar.zst`，相比 `gzip -9` 体积约 -30%)。机器人未安装 `zstd` 时自动回退到 `pigz -9` 产出 `.tar.gz`。

如需强制指定算法 (例如紧急回退 / 兼容性问题)，可通过环境变量 `KUAVO_CRASH_COMPRESSOR` 控制:

```bash
KUAVO_CRASH_COMPRESSOR=gzip ./tools/crash-report/CrashReport.sh ~/.ros/stdout/<TS>/
KUAVO_CRASH_COMPRESSOR=zstd ./tools/crash-report/CrashReport.sh ~/.ros/stdout/<TS>/  # 强制 zstd，未装则报错
```

分析端 `docs_internal/kuavo-gdb/kuavo-gdb.sh` 通过 magic byte 自动识别两种格式，无需关注后缀差异。

### 已知问题须知

- 如果您根据自己的的需求改动了代码，可能会导致乐聚人员在分析崩溃文件时无法对应到您的改动。
- 如果您在本地新的`git commit`，可能会导致乐聚人员无法对应到您的改动。
- 如果您在发生崩溃后又多次运行，可能会导致分析失败。