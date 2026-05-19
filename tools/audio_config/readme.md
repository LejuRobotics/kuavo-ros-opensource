# 音频配置

`deploy_eb03_pulseaudio_fix.sh` 用于部署 EB03 USB 音频修复配置。

作用：

1. 启用目标用户的 `pulseaudio` 服务
2. 将 ALSA 默认输出切到 `EB03_LJ`
3. 安装开机自动修复服务 `eb03-pulseaudio-fix.service`

## 使用方法

```bash
sudo tools/audio_config/deploy_eb03_pulseaudio_fix.sh
sudo reboot
```


## 检查是否生效

```bash
systemctl status eb03-pulseaudio-fix.service --no-pager
pactl info
aplay -l
```

如果需要看日志：

```bash
journalctl -u eb03-pulseaudio-fix.service --no-pager
```
