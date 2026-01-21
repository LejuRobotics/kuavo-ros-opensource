
### 1. 检测雷达 (check_lidar)

**请求格式：**
```json
{
    "cmd": "check_lidar",
    "data": {}
}
```

**响应格式：**
```json
{
    "cmd": "check_lidar",
    "data": {
        "code": 0,
        "message": "Lidar check completed successfully",
        "lidar_active": true
    }
}
```

**错误响应：**
```json
{
    "cmd": "check_lidar",
    "data": {
        "code": 1,
        "message": "Lidar topic /livox/cloud not found",
        "lidar_active": false
    }
}
```

---

### 2. 新建地图 (create_map)

**请求格式：**
```json
{
    "cmd": "create_map",
    "data": {
        "map_name": "example_map"
    }
}
```

**响应格式：**
```json
{
    "cmd": "create_map",
    "data": {
        "code": 0,
        "message": "Map creation command sent for: example_map",
        "map_name": "example_map"
    }
}
```

**错误响应：**
```json
{
    "cmd": "create_map",
    "data": {
        "code": 1,
        "message": "Map name is required"
    }
}
```

---

### 3. 保存地图 (save_map)

**请求格式：**
```json
{
    "cmd": "save_map",
    "data": {}
}
```

**响应格式：**
```json
{
    "cmd": "save_map",
    "data": {
        "code": 0,
        "message": "地图保存成功"
    }
}
```

**错误响应：**
```json
{
    "cmd": "save_map",
    "data": {
        "code": 1,
        "message": "无法连接到建图服务: Service not available"
    }
}
```

---

### 4. 结束建图 (stop_mapping)

**请求格式：**
```json
{
    "cmd": "stop_mapping",
    "data": {}
}
```

**响应格式：**
```json
{
    "cmd": "stop_mapping",
    "data": {
        "code": 0,
        "message": "建图已停止"
    }
}
```

**错误响应：**
```json
{
    "cmd": "stop_mapping",
    "data": {
        "code": 1,
        "message": "无法连接到建图服务: Service not available"
    }
}
```

---

### 5. 2D地图显示 (map_update)

**说明：** 此接口为服务端主动推送，客户端无需发送请求

**推送格式：**
```json
{
    "cmd": "map_update",
    "data": {
        "code": 0,
        "message": "Map data updated",
        "map_data": "base64编码的PNG图片数据",
        "width": 1024,
        "height": 768,
        "resolution": 0.05,
        "origin": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "timestamp": 1635678901.234
    }
}
```

---

### 7. 查询机器人是否站立 (check_robot_standing)

**请求格式：**
```json
{
    "cmd": "check_robot_standing",
    "data": {}
}
```

**响应格式：**
```json
{
    "cmd": "check_robot_standing",
    "data": {
        "code": 0,
        "message": "Robot is standing (status: standing)",
        "is_standing": true
    }
}
```

**错误响应：**
```json
{
    "cmd": "check_robot_standing",
    "data": {
        "code": 1,
        "message": "Failed to check robot standing status: Connection error"
    }
}
```

---

### 8. 辅助扫描 (assist_scan)

**请求格式：**
```json
{
    "cmd": "assist_scan",
    "data": {}
}
```

**响应格式：**
```json
{
    "cmd": "assist_scan",
    "data": {
        "code": 0,
        "message": "Head assist scan completed (left-right-center movement)"
    }
}
```

**错误响应：**
```json
{
    "cmd": "assist_scan",
    "data": {
        "code": 1,
        "message": "Failed to perform assist scan: Head publisher not initialized"
    }
}
```

---

### 9. 编辑地图 (edit_map)

**请求格式：**
```json
{
    "cmd": "edit_map",
    "data": {
        "map_name": "example_map",
        "operation": "fill/clear",
        "points": [100, 200, 100, 150, 200, 200, 200, 150]//(x1,y1,x2,y2,x3,y3,x4,y4)
    }
}
```

**响应格式：**
```json
{
    "cmd": "edit_map",
    "data": {
        "code": 0,
        "message": "地图编辑成功",
        "map_name": "example_map",
        "map_image": "base64编码的PNG图片数据",
        "map_info": {
            "width": 1024,
            "height": 768,
            "resolution": 0.05,
            "origin": {"x": -10.0, "y": -10.0, "z": 0.0}
        }
    }
}
```

**错误响应：**
```json
{
    "cmd": "edit_map",
    "data": {
        "code": 1,
        "message": "必须指定地图名称"
    }
}
```

---

### 10. 给出地图列表 (get_all_maps)

**请求格式：**
```json
{
    "cmd": "get_all_maps",
    "data": {}
}
```

**响应格式：**
```json
{
    "cmd": "get_all_maps",
    "data": {
        "code": 0,
        "message": "Found 3 maps using MapLists format",
        "maps": ["map1", "map2", "map3"]
    }
}
```

**错误响应：**
```json
{
    "cmd": "get_all_maps",
    "data": {
        "code": 0,
        "message": "No map lists topic found (/map_lists not published)",
        "maps": []
    }
}
```
### 11. 重命名地图 (rename_map)

**请求格式：**
```json
{
    "cmd": "rename_map",
    "data": {
        "old_name": "map_2024-01-15_14-30-25",
        "new_name": "living_room_map"
    }
}
```

**响应格式：**
```json
{
    "cmd": "rename_map",
    "data": {
        "code": 0,
        "message": "地图重命名成功: map_2024-01-15_14-30-25 -> living_room_map",
        "old_name": "map_2024-01-15_14-30-25",
        "new_name": "living_room_map"
    }
}
```

**错误响应：**
```json
{
    "cmd": "rename_map",
    "data": {
        "code": 1,
        "message": "原地图名称和新地图名称不能为空",
        "old_name": "",
        "new_name": ""
    }
}
```

---

### 12. 删除地图 (delete_map)

**请求格式：**
```json
{
    "cmd": "delete_map",
    "data": {
        "map_name": "living_room_map"
    }
}
```

**响应格式：**
```json
{
    "cmd": "delete_map",
    "data": {
        "code": 0,
        "message": "地图删除成功: living_room_map",
        "map_name": "living_room_map"
    }
}
```

```json
{
    "cmd": "delete_map",
    "data": {
        "code": 1,
        "message": "无法删除当前正在使用的导航地图",
        "map_name": "living_room_map"
    }
}
```

```json
{
    "cmd": "delete_map",
    "data": {
        "code": 2,
        "message": "地图名称不能为空",
        "map_name": ""
    }
}
```

```json
{
    "cmd": "delete_map",
    "data": {
        "code": 3,
        "message": "地图删除失败: living_room_map",
        "map_name": "living_room_map"
    }
}
```

---

### 13. 标记任务点 (task_point)

**请求格式：**
```json
{
    "cmd": "task_point",
    "data": {
        "operation": 0,
        "task_point": {
            "pose": {
                "position": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0
                }
            },
            "name": "home_position"
        },
        "name": "home_position",
        "use_robot_current_pose": false
    }
}
```

**响应格式：**
```json
{
    "cmd": "task_point",
    "data": {
        "code": 0,
        "message": "任务点操作成功",
        "success": true
    }
}
```

**错误响应：**
```json
{
    "cmd": "task_point",
    "data": {
        "code": 1,
        "message": "任务点操作失败: 服务不可用",
        "success": false
    }
}
```

---

### 14. 获取任务点列表 (get_task_points)

**请求格式：**
```json
{
    "cmd": "get_task_points",
    "data": {}
}
```

**响应格式：**
```json
{
    "cmd": "get_task_points",
    "data": {
        "code": 0,
        "message": "获取任务点列表成功",
        "task_points": [
            {
                "name": "home_position",
                "pose": {
                    "position": {
                        "x": 1.0,
                        "y": 2.0,
                        "z": 0.0
                    },
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 1.0
                    }
                }
            }
        ]
    }
}
```

**错误响应：**
```json
{
    "cmd": "get_task_points",
    "data": {
        "code": 1,
        "message": "获取任务点列表失败: 服务不可用",
        "task_points": []
    }
}
```

---

### 15. 导航到任务点 (navigate_to_task_point)

**请求格式：**
```json
{
    "cmd": "navigate_to_task_point",
    "data": {
        "task_name": "home_position"
    }
}
```

**响应格式：**
```json
{
    "cmd": "navigate_to_task_point",
    "data": {
        "code": 0,
        "message": "开始导航到任务点: home_position",
        "success": true
    }
}
```

**错误响应：**
```json
{
    "cmd": "navigate_to_task_point",
    "data": {
        "code": 1,
        "message": "导航失败: 找不到指定的任务点",
        "success": false
    }
}
```




---

### 16. 获取机器人当前位置 (get_robot_position)

**请求格式：**
```json
{
    "cmd": "get_robot_position",
    "data": {}
}
```

**响应格式：**
```json
{
    "cmd": "get_robot_position",
    "data": {
        "code": 0,
        "message": "Robot position retrieved successfully",
        "position": {
            "x": 1.234,
            "y": 2.567,
            "z": 0.0
        },
        "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 1.0
        },
        "linear_velocity": {
            "x": 0.1,
            "y": 0.0,
            "z": 0.0
        },
        "angular_velocity": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.1
        },
        "frame_id": "odom",
        "child_frame_id": "livox_frame",
        "timestamp": 1234567890.123
    }
}
```

**错误响应：**
```json
{
    "cmd": "get_robot_position",
    "data": {
        "code": 1,
        "message": "No odometry data available yet"
    }
}
```

---

### 17. 基于任务点的校准 (calibration_by_task_point)

**请求格式：**
```json
{
    "cmd": "calibration_by_task_point",
    "data": {
        "task_point_name": "home_position"
    }
}
```

**响应格式：**
```json
{
    "cmd": "calibration_by_task_point",
    "data": {
        "code": 0,
        "message": "基于任务点 'home_position' 的校准成功",
        "task_point_name": "home_position"
    }
}
```

**错误响应：**
```json
{
    "cmd": "calibration_by_task_point",
    "data": {
        "code": 1,
        "message": "基于任务点 'home_position' 的校准失败: 任务点不存在"
    }
}
```
---

### 18. 加载地图 (load_map)

**请求格式：**
```json
{
    "cmd": "load_map",
    "data": {
        "map_name": "example_map"
    }
}
```

**响应格式：**
```json
{
    "cmd": "load_map",
    "data": {
        "code": 0,
        "msg": "Map loaded successfully",
        "map_path": "/path/to/maps/example_map.png"
    }
}
```

**错误响应：**
```json
{
    "cmd": "load_map",
    "data": {
        "code": 2,
        "msg": "Service `load_map` call failed: Service not available"
    }
}
```