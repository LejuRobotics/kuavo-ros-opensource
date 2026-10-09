# kuavobrain 中 ROBOT_VERSION 使用分析

## 概述

kuavobrain 对 `ROBOT_VERSION` 的使用主要集中在 `kuavo_websocket_service` 模块，用于数据溯源（bag 文件记录机器人版本）和 API 上报。不涉及路径拼接或版本条件分支，影响面较小。

---

## 使用场景

### 1. 读取 ROS 参数 `/robot_version`

**文件**：`src/kuavo_websocket_service_opensource/src/kuavo_websocket_service/fileds.py`

```python
@property
def robot_version(self):
    # 读取 ROS 参数 /robot_version，skip_check 模式下返回 "00"
    return rospy.get_param("/robot_version", "00")
```

当前假设 `/robot_version` 是数字字符串（如 `"45"`）。

**迁移影响**：
- 如果 `/robot_version` 保持 int 不变（当前方案），这里不受影响
- 新代码可以改为读 `/robot_version_next` string 参数

---

### 2. Bag 文件元数据中记录版本号

**文件**：
- `src/kuavo_websocket_service_opensource/src/kuavo_websocket_service/bag_processor.py`
- `src/kuavo_websocket_service_opensource/src/kuavo_websocket_service/data_handlers.py`

版本号作为 bag 文件的元数据记录到 SQLite 数据库，用于数据溯源：

```python
# data_handlers.py
robot_version = self._kuavo_config.robot_version  # 读取 "45"
# 传入 bag_processor
start_recording(..., robot_version=robot_version)

# bag_processor.py
# 写入数据库
bag.robot_version: robot_version
```

**迁移影响**：这里只是**透传存储**版本号字符串，不做任何解析或判断。无论值是 `"45"` 还是 `"kuavo-4pro-biped-revo1hand"` 都能正常存储。不需要改动。

---

### 3. 上传版本号到服务端 API

**文件**：
- `src/kuavo_websocket_service_opensource/src/kuavo_websocket_service/bag_upload.py`

Bag 上传时将 `robot_version` 作为 `lowerVersion` 字段发送给服务端：

```python
"lowerVersion": single_bag_info.get(bag.robot_version, "")
```

**迁移影响**：同样是透传，不做解析。但服务端可能对 `lowerVersion` 字段有格式假设（如期望数字）。迁移时需确认服务端是否能接受新格式字符串。

---

### 4. 数据库表结构中的版本字段

**文件**：`src/kuavo_websocket_service_opensource/src/kuavo_websocket_service/data_base.py`

SQLite 表中有 `robotVersion` 和 `topicsVersion` 列（TEXT 类型）：

```python
{bag.robot_version} TEXT DEFAULT ''
```

**迁移影响**：TEXT 类型，不受值格式影响。不需要改动。

---

### 5. 文档中的版本引用

**文件**：
- `src/kuavo_websocket_service_opensource/src/kuavo_data_challenge/docs/deployment/real_eval.md`
- `src/manipulation_nodes/handcontrollerdemorosnode/README.md`

文档中提到 `ROBOT_VERSION` 环境变量、`TotalMassV${ROBOT_VERSION}`、`kuavo_v$ROBOT_VERSION/kuavo.json` 等用法，是对 kuavo-ros-control 用法的引用说明。

**迁移影响**：文档更新，非代码改动。

---

### 6. 其他版本概念（非 ROBOT_VERSION）

kuavobrain 中还有几个与 `ROBOT_VERSION` 无关的版本概念，不受迁移影响：

| 版本类型 | 用途 | 是否受影响 |
|---------|------|:---------:|
| `deb_version` | Debian 包版本号（从 package.xml 解析），用于自动更新检查 | 否 |
| `topics_version` | ROS Topics 配置版本（从 JSON 读取），嵌入 bag 文件名 | 否 |
| `model_version` | AI 模型版本（从服务端下载），用于模型管理 | 否 |

---

## 影响总结

| 场景 | 文件数 | 迁移难度 |
|------|:------:|:--------:|
| 读取 `/robot_version` ROS 参数 | 1 | 低（保持 int 不动则无需改） |
| Bag 元数据透传存储 | 2 | 无需改动（纯字符串存储） |
| API 上传 `lowerVersion` | 1 | 低（需确认服务端是否接受新格式） |
| 数据库 TEXT 字段 | 1 | 无需改动 |
| 文档引用 | 2 | 文档更新 |

**总体影响极小**。kuavobrain 不做版本解析和条件分支，只是透传和存储版本号字符串。核心风险点只有一个：API 上传的 `lowerVersion` 字段，需确认服务端对该字段的格式要求。
