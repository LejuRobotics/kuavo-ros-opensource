# 手臂碰撞包 kuavo_arm_collision_check 用于检测手臂碰撞

启动方式为 `roslaunch kuavo_arm_collision_check arm_collision_check.launch`

该节点启动后根据 robot_version 读取 urdf 文件，根据该文件加载 stl 模型，优先使用 cache 文件夹内的缓存模型，其次使用 meshes 文件夹内的曲面模型（**长手灵巧手**机器人版本），该文件夹的文件经过减面和模型内部掏空处理，能够减小计算量。如果 meshes 内没有 stl 文件则使用 kuavo_assets 内的 stl 文件。

程序第一次加载会使用 openmesh 库将 stl 模型碰撞，并将处理后的文件存储在 cache 缓存文件夹内，后续启动该节点将使用 cache 内的 stl 模型，为了防止代码修改导致缓存模型失效被加载，**编译将清空 cache 文件夹**。

## 工作流程

- 加载 stl 文件
- 将 stl 文件内的模型添加为 fcl 的碰撞体
- 订阅 tf，更新碰撞体的位姿
- 按照一定频率（默认 5hz）计算碰撞，发生碰撞向 /arm_collision_info 发布数据，数据包含涉及碰撞的关节名， msg 为（kuavo_msgs/msg/armCollisionCheckInfo.msg）。同时发布计算碰撞的耗时 /collision_check_duration，单位为毫秒。根据启动节点时的 ROS 参数发布 /collision_markers 用于在 rviz 显示碰撞 marker。


## 节点参数

- /arm_collision_freq 节点运行时计算碰撞的频率，默认为 5。
- /publish_collision_markers 发生碰撞时是否发布 maker，默认不发布