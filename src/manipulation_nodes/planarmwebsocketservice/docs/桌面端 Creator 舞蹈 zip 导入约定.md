# 桌面端 Creator 舞蹈 zip 导入约定

机器人侧通过 ws 服务 `plan_arm_action_websocket_server` 提供 cmd `import_creator_dance`。桌面端把 Leju Creator 导出的舞蹈 zip scp 到约定暂存目录,然后调用该 cmd 完成解压、控制器文件落盘、`rl_controllers.yaml` 注册。

## 设计原则:服务端只管控制器,前端管按键绑定与音乐

**本接口职责单一**:解压 zip → 写控制器文件(`dance_param_<controller>.info` / `<controller>.onnx` / `<controller>.csv`)→ 在 `rl_controllers.yaml` 里追加一条 `DANCE_CONTROLLER` 条目 → 把 `展示名 → 控制器名` 写进映射文件。**不动 `customize_config.json`,不动任何 wav 文件**(即便 zip 里携带 wav,服务端也一律忽略)。

### 关键:展示名(可中文) vs 控制器名(纯英文,内部用)

控制器名同时被当成 **ROS 服务名段**(`/humanoid_controller/<controller>/restart_dance`)和 **info/onnx/csv 文件名前缀**,因此**必须是纯 ASCII 标识符**;中文名会让 ROS `advertiseService` 抛异常、控制器按字节精确匹配失败(日志里中文渲染成 `?????????`)。

为了兼顾"用户想看中文"和"控制器必须英文":

- **展示名 `dance_name`**:= zip 文件名去掉 `.zip` 后缀,**允许中文 / 英文 / 数字**(如 `爱情鸟`)。这是写进 `customize_config.json` 给用户看的可读名。
- **控制器名 `controller`**:由服务端从展示名**自动派生**的纯英文名(形如 `dance_<可选英文片段>_<hash>`,如 `dance_e36aef9d91`)。**前端无需关心、也无需填写**。
- **映射文件** `creator_dance_upload/dance_name_map.json` 固化两者对应关系,joy 节点运行时据此把 json 里的中文 `dance_name` 换成英文控制器名再去切换。

桌面端负责:
- 给 zip 起一个**对用户可读**的名字(`<展示名>.zip`),展示名即 json 里的 `dance_name`,可中文。
- 按键绑定:自己改写 `customize_config.json` 中目标 key 的 `type`/`dance_name`/`music_name` 字段,`dance_name` 填**展示名**(= zip 去后缀名)。
- 音乐:wav scp 到 `/home/lab/.config/lejuconfig/music/`(或 `get_robot_info.music_folder_path`)。zip 里的 wav 不会被解出来 —— **不要依赖**。

### `customize_config.json` 字段类型(写入时务必匹配)

| 字段 | 类型 | 示例 |
|---|---|---|
| `type` | **string** | `"dance"` |
| `dance_name` | **string**(**不是数组**) | `"爱情鸟"` |
| `music_name` | **string[]** | `["爱情鸟.wav"]` |
| `arm_pose_name` | **string[]**(仅 action 类型用) | `["双手叉腰"]` |

`dance_name` 写成数组(例如 `["爱情鸟"]`)会被 joy 节点当作非法值,按键将不响应。完整 dance 类型 key 的形态:

```json
"customize_action_M2_B": {
  "type": "dance",
  "dance_name": "爱情鸟",
  "music_name": ["爱情鸟.wav"]
}
```

> **重要**:`dance_name` 必须与 zip 去掉 `.zip` 后缀的名字**按 Unicode NFC 归一化后逐字节一致**(含多点、前后空格、大小写)。服务端写映射表、joy 查表都会先做 `NFC + strip`,但前端写 json 时请直接用与 zip 名相同的字符串,避免 macOS(NFD)/输入法差异导致查表 miss。

### joy 运行时解析规则(回退直达)

joy 切换舞蹈时:`normalize(dance_name)` 在映射表里查到 → 用对应英文控制器名;**查不到 → 直接用 `dance_name` 原值**去切换。

这意味着:
- **走本接口上传的舞蹈**:json 写中文展示名,joy 查表换成英文名,正常工作。
- **手动绑定 / 出厂预装舞蹈**:可以直接在 json 里把 `dance_name` 写成**英文控制器名**(如 `dance_balei`),joy 查不到映射就原值直达,同样工作。
- 非法 / 不存在的名字最终由控制器侧拒绝(joy 不报死,只是切换失败)。

## zip 内容约定

zip 里须含 **yaml(env 配置)、onnx(模型)、csv(轨迹)各恰好一个**;**文件名不限**(Creator 导出的是 md5 随机串,也兼容固定的 `env.yaml`/`model.onnx`/`trajectory.csv`),服务端**按扩展名**定位。某类缺失或出现多个,均以 `E_BAD_ZIP` 报错。zip 里的 wav 一律忽略。

## 整体流程

```
[桌面端]
   │ 1. scp <展示名>.zip → ~/.config/lejuconfig/creator_dance_upload/
   │
   │ 2. ws cmd: import_creator_dance   (只传 zip_filename [+ force])
   │
[机器人]
   handler.py
     └→ subprocess: tools/import_creator_dance/import_creator_dance.py <zip> --json [--force]
        ├ 解压 zip,生成 dance_param_<controller>.info / <controller>.onnx / <controller>.csv
        └ 写 rl_controllers.yaml (注册 DANCE_CONTROLLER)
     └→ 把 展示名 → 控制器名 写进 dance_name_map.json
   handler 不删除暂存 zip(保留原包, 由前端按需清理)。

[桌面端] 3. 自行 scp wav 到 music 目录, 自行改写 customize_config.json(dance_name=展示名)
[桌面端] 4. 触发 joy 重载(/update_joy_customize_config), joy 同时重载 json 与映射表
```

> **时序要求**:必须 **先调 `import_creator_dance`(写映射表)→ 再替换仓库 json → 再触发 joy 重载**。顺序颠倒(json/重载早于映射表写入)会让 joy 查不到映射而回退原中文名,切换失败。

## 暂存目录

实际路径**不要硬编码**,从 `get_robot_info` 拿:

```json
// 请求
{"cmd": "get_robot_info", "data": {}}

// 响应里包含
{
  "cmd": "get_robot_info",
  "data": {
    "code": 0,
    "music_folder_path": "/home/lab/.config/lejuconfig/music",
    "creator_dance_upload_path": "/home/lab/.config/lejuconfig/creator_dance_upload"
  }
}
```

ws 服务启动时会自动 `mkdir -p` 这个目录,桌面端只管 scp 文件进去。映射文件 `dance_name_map.json` 也落在该目录下,由服务端维护,前端只读不写。

## ws cmd: `import_creator_dance`

### 请求

```json
{
  "cmd": "import_creator_dance",
  "data": {
    "zip_filename": "爱情鸟.zip",
    "force": true
  }
}
```

| 字段 | 必填 | 说明 |
|---|---|---|
| `zip_filename` | 是 | 相对暂存目录的文件名,不带路径;去掉 `.zip` 后即为**展示名**(= json 的 `dance_name`,可中文) |
| `force` | 否,默认 `false` | 同名 controller 或目标 info/onnx/csv 已存在时是否覆盖 |

### force 字段说明

`force` 仅控制"已存在则覆盖"语义,**不绕过**任何文件 / 关节数 / zip 合法性校验。

不带 `force` 时,遇到以下情况会以 `E_BINDING_EXISTS` 拒绝:
- `rl_controllers.yaml` 里已有同名 controller;
- 目标控制器的 `.info` / `.onnx` / `.csv` 已存在(说明此前导入过同展示名的舞蹈)。

控制器名由展示名确定性派生,因此**同一展示名重复导入会落到同一控制器、同一组文件**;`force: true` 即覆盖更新("后者覆盖前者")。

**推荐桌面端默认传 `force: true`**:上传舞蹈本身是用户主动覆盖意图,不带 force 会让"换同名舞蹈"场景每次先 fail 再重发,产生不必要往返。

### 成功响应

```json
{
  "cmd": "import_creator_dance",
  "data": {
    "code": 0,
    "dry_run": false,
    "robot_version": "17",
    "dance_name": "爱情鸟",
    "controller": "dance_e36aef9d91",
    "info_path": ".../rl/dance_param_dance_e36aef9d91.info",
    "onnx_path": ".../model/networks/dance_e36aef9d91.onnx",
    "csv_path": ".../rl/dance_e36aef9d91.csv",
    "controller_existed": false
  }
}
```

字段含义:
- `dance_name`:展示名(= zip 去 `.zip` 后缀,归一化后)。**前端写 json 的 `dance_name` 就用它。**
- `controller`:服务端派生 + 落盘 + 注册的英文控制器名,并已写入映射表。前端**不需要**写进 json(写了也行,joy 查不到会原值直达;但推荐写展示名)。
- `controller_existed`:之前 `rl_controllers.yaml` 是否已有同名 controller。`true` 表示覆盖更新,`false` 表示新增。

### 失败响应

```json
{
  "cmd": "import_creator_dance",
  "data": {
    "code": 2,
    "error": "...原始错误消息...",
    "error_kind": "E_BINDING_EXISTS"
  }
}
```

### 错误码 `error_kind`

| code | 含义 | 典型触发 |
|---|---|---|
| `E_BAD_ZIP` | zip 不存在 / 损坏 / 内容缺失 | 暂存目录里找不到 zip;zip 内缺少或出现多个 yaml/onnx/csv(各须恰好一个);trajectory 列数不对 |
| `E_JOINT_MISMATCH` | zip 导出关节数与当前机器人 `ROBOT_VERSION` 模板不匹配 | 用错了机器人版本对应的 Creator 导出 |
| `E_BINDING_EXISTS` | 同名 controller 或同名落盘文件已存在,且未传 `force` | 见上"force 字段说明" |
| `E_GENERIC` | 其他(`ROBOT_VERSION` 未设置、控制器名哈希冲突等) | 看 `error` 字段 |

> 哈希冲突(`E_GENERIC` + `error` 含"命名冲突"):极小概率两个不同展示名派生出同一控制器名,服务端会拒绝以免覆盖另一支舞,提示改 zip 名即可。

## 桌面端建议流程

1. **(首次连接 / 启动)** 调 `get_robot_info` 拿 `creator_dance_upload_path` 和 `music_folder_path`,缓存到本次会话。
2. 给 zip 起一个**对用户可读**的 `<展示名>.zip` 名字(可中文),scp 到 `creator_dance_upload_path`。
3. (可选 / 解耦)若需要同步换音乐,把 wav scp 到 `music_folder_path`。
4. 调 `import_creator_dance`,**推荐 `force: true`**;拿到 `code: 0` 后记下返回的 `dance_name`。
5. 用桌面端自己的 json 写入通路改写 `customize_config.json` 中目标 key 的 `type` / `dance_name`(= 上一步的展示名)/ `music_name`。
6. 触发 joy 重载(发布 `/update_joy_customize_config`),joy 会**同时重载 json 与映射表**。
7. 提示用户:**新注册的 controller 真正可切需要重启 / 重载 humanoid controller**(`rl_controllers.yaml` 重新加载后才在控制器侧存在)。

## 备注

- 服务端**不删除**暂存 zip(保留原包便于复用/排查/重导, 需要清理由前端自理);映射表 `dance_name_map.json` 由服务端维护(读-改-原子写)。
- `ROBOT_VERSION` 由 systemd 注入(`roban_joy_monitor.service`),桌面端无需关心。如果某机器人没正确部署,会以 `E_GENERIC + error: "ROBOT_VERSION is not set..."` 返回。
- 本接口当前是**同步阻塞**的:整个流程 1~3 秒,桌面端拿到响应即代表完成。
- 解绑 / 换新展示名覆盖后,旧控制器条目、旧 info/onnx/csv、旧映射表 entry 可能残留(不影响功能),清理后续再补。
