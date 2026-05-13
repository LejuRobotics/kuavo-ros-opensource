# USB扬声器自动禁用麦克风脚本

## 背景问题

    在使用 USB 外置扬声器时，在 Linux 系统中会被错误识别为“带麦克风的音频设备”，但根本没有麦克风硬件。导致系统创建无效的麦克风输入源，在需要正常使用麦克风时可能误选该“假麦克风”，造成无声或回音；

## 脚本作用

    扫描所有已连接的 USB 声卡。
    检查其是否支持 output:analog-stereo 模式。
    支持，则强制将其 profile 设为 output:analog-stereo（纯输出）。
    跳过真正的麦克风或耳机设备。

## 建议在插入 USB 扬声器后运行一次

## 赋予执行权限

    chmod +x snd_usb_audio.sh

## 运行脚本

    ./snd_usb_audio.sh