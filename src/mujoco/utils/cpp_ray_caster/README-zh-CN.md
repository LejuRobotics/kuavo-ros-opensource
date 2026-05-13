# 迁移说明

## 代码移植

将 RayCastering Famliy 的代码放在 mujoco 下，例如新建 utils/cpp_ray_caster

```bash
kuavo-robot-deploy/src/mujoco/utils$ tree
.
└── cpp_ray_caster
    ├── Noise.hpp
    ├── RayCasterCamera.cpp
    ├── RayCasterCamera.h
    ├── RayCaster.cpp
    ├── RayCaster.h
    ├── RayCasterLidar.cpp
    ├── RayCasterLidar.h
    └── RayNoise.hpp
```

## 修改 CMakeList.txt

添加

```
set(ray_caster_path ${CMAKE_CURRENT_SOURCE_DIR}/utils/cpp_ray_caster)
include_directories(${ray_caster_path})
file(GLOB ray_caster ${ray_caster_path}/*.cpp)
```

修改

```
add_library(${PROJECT_NAME} SHARED src/mujoco_node.cc ${SIMULATE_SRCS} ${LCM_SRCS} ${ray_caster})
target_link_libraries(${PROJECT_NAME} PUBLIC ${MUJOCO_LIB} ${catkin_LIBRARIES} Threads::Threads glfw ${glfw3_LIBRARIES} ${LCM_LIBRARIES} Eigen3::Eigen)
```

## 在 XML 中添加相机

以 Kuavo-S46 为例，在 base_link 下添加一个子节点

```xml
<body name="waist_camera_link" pos="0.2 0 0" quat="0.868 0 0.497 0">  <!-- 绕 y 轴右手系正向旋转 58.588 度 -->
   <inertial pos="0 0 0" mass="0.01" diaginertia="1e-05 1e-05 1e-05" />
   <camera name="waist_camera" pos="0 0 0" xyaxes="0 -1 0 0 0 1" fovy="80" />
</body>
```

## 核心步骤和 API

1. 确认相机参数
   - focal_length
   - horizontal_aperture
   - num_h_ray
   - num_v_ray
   - {distance_range}
   - vertical_aperture (optional)
2. 在 mujoco 仿真循环开始前初始化相机
3. compute_distance 用于计算射线打到的位置

## 相机参数

1. focal_length=2.12
2. horizontal_aperture=4.24
3. num_h_ray=64
4. num_v_ray=36
5. distance_range={0.1, 10.0}
6. vertical_aperture=2.4480

# RayCaster Family

## RayCaster
**resolution:real(1),“0”**     
&emsp;分辨率

**size:real(2),“0 0”**     
&emsp;尺寸 米

**type:[base,yaw,world]”**     
&emsp;base 自坐标系相机lookat
&emsp;yaw 自坐标系yaw,世界z向下
&emsp;world 世界坐标系z向下


## RayCasterCamera
**focal_length:real(1),“0”**     
&emsp;焦距 cm

**horizontal_aperture:real(1),“0”**     
&emsp;画面水平尺寸 cm

**vertical_aperture:real(1),“0”**     
&emsp;画面垂直尺寸 cm

**size:real(2),“0 0”**     
&emsp;h_ray_num,v_ray_num


## RayCasterLidar
**fov_h:real(1),“0”**     
&emsp;fov_h 角度

**fov_v:real(1),“0”**     
&emsp;fov_v 角度

**size:real(2),“0 0”**     
&emsp;h_ray_num,v_ray_num
