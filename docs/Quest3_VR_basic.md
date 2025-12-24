# 介绍

在使用遥操作的时候我们需要用到 Quest3 的设备来捕捉用户的动作控制机器人来搜集数据。在这个文档中我们会介绍在使用 Quest3 的过程中会遇到的几个问题或者需要用到的操作

APK 下载地址 [Quest3 APK 地址](https://kuavo.lejurobot.com/Quest_apks/leju_kuavo_hand-0.0.1-178-ge46a1e5.apk "Quest3 APK 地址")

# 安装 SideQuest

如果无法下载，可以使用我们提供的源来安装。我们提供的地址是：[SideQuest下载地址](https://kuavo.lejurobot.com/Quest_apks/SideQuest-Setup-0.10.42-x64-win.exe "SideQuest 下载地址")，请注意这个源不会和官方的版本同步更新，目前同步的日期是 2024/11/22。请大家如果在发现版本距离久的时候自行下载软件。

基本使用请参考官方视频，或者 Bilili 的视频： [SideQuest的基本使用](https://www.bilibili.com/video/BV1uY41157Ki/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "SideQuest基本使用")

如何在 Quest3 里面授权的视频: [Quest3 允许授权](https://www.bilibili.com/video/BV1zzBiYqE8m/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "Quest3 允许授权")

# 如何更新 Quest3 的系统固件

目前已知的是 V68 的系统固件版本会存在卡顿和遥控器定位的问题。大家的设备如果是 V68 需要升级系统固件。

升级系统固件需要把 Quest3 连接在一个可以访问 meta 服务器网络的环境下。

具体升级的方法请参考: [Quest3 更新系统固件](https://www.bilibili.com/video/BV1FBBiYMEp4/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "Quest3 更新系统固件")

# 如何启动已安装的程序

手柄: [如何启动程序-手柄](https://www.bilibili.com/video/BV1EBBiYKE9B/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "如何启动程序-手柄")

手势识别: [如何启动程序-手势](https://www.bilibili.com/video/BV1JBBiYMEmK/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "如何启动程序-手势")

# 如何导出已录制的视频

在我们技术支持的时候工程师可能需要大家录制一段在软件里面的操作画面然后发送给我们。

录制视频的方法请参考：[Quest3 录制视频](https://www.bilibili.com/video/BV1U7411p7h2/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "Quest3 录制视频")

导出视频的方法请参考: [Quest3 录屏分享](https://www.bilibili.com/video/BV1fzBiYiEa2/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "Quest3 录屏分享")

# 如何去除空间限制

默认 Quest3 会需要用户建立一个虚拟空间，当你操作 VR 的时候走出这个空间就会自动暂停我们的程序提示回到虚拟空间上。在展馆之类的场景的时候就有限制。如果想要去掉这个限制，可以参考以下视频。注意：按照视频中说明操作之后主界面的 paththrough 会不起作用，但是进入程序里面是可以透视的，不影响使用。

参考视频：[Quest3 突破空间限制](https://www.bilibili.com/video/BV1iYzwYqEwt/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "Quest3 突破空间限制")

如果是 V72 之后的系统，设置界面可能有变化，可以参考

参考视频：[Quest3 在 V72 以上的版本如何突破空间边界限制](https://www.bilibili.com/video/BV11KXjYtE8v/?share_source=copy_web&vd_source=2d815abfceff1874dd081e6eb77cc262 "Quest3 突破空间限制")

# 如何实时查看到 Quest3 投屏的屏幕

1. 先完成授权的步骤，请参考前面: <如何在 Quest3 里面授权的视频>

2. 根据自己的系统安装 

- [scrcpy-linux-x86_64-v3.1.tar.gz](https://kuavo.lejurobot.com/statics/scrcpy-linux-x86_64-v3.1.tar.gz)
- [scrcpy-macos-aarch64-v3.1.tar.gz](https://kuavo.lejurobot.com/statics/scrcpy-macos-aarch64-v3.1.tar.gz)
- [scrcpy-macos-x86_64-v3.1.tar.gz](https://kuavo.lejurobot.com/statics/scrcpy-macos-x86_64-v3.1.tar.gz)
- [scrcpy-win64-v3.1.zip](https://kuavo.lejurobot.com/statics/scrcpy-win64-v3.1.zip)

3. 各自解压之后在对应的目录下启动终端，确保当前电脑已经通过 USB 连接 Quest3, 并且 Quest3 已经开机

4. 执行对应的 scrcpy 指令

示例视频

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=113683724243013&bvid=BV1kAk2Y1Edm&cid=27433897643&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"></iframe>

# 如何查看 Quest3 和 Kuavo 的网络延迟

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=114380415047614&bvid=BV1wz5XzBEKf&cid=29549135877&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"></iframe>

