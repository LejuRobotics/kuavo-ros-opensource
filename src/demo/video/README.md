
动作文件的路径为：`/home/lab/.config/lejuconfig/action_files/xxx.tact`

## 场景 1 和 2 的启动方法
```
source devel/setup.bash  
roslaunch humanoid_controllers  load_kuavo_real.launch
```

```
source devel/setup.bash 
roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch
```


```
source devel/setup.bash
roslaunch planarmwebsocketservice plan_arm_action_websocket_server.launch 
```


```
source devel/setup.bash
python3 ./src/demo/video/bump_salute.py
```  




`src/demo/video/hit_hand.py` 修改方式：主要修改 `duration` 控制运动的时间。

## 搬箱子的启动方法  
```
source devel/setup.bash  
roslaunch humanoid_controllers  load_kuavo_real.launch
```

```
source devel/setup.bash 
roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch arm_restore_flag:=false
```


```
source devel/setup.bash
python3 /src/demo/video/turn_move_box_joy.py
```

`src/demo/video/turn_move_box.py` 修改方式：主要修改 `duration` 控制运动的时间。


## 导航键图
- IP 修改  

```
    sudo nmcli connection modify netplan-enx00e04c68345c ipv4.addresses 192.168.1.102/24
    sudo nmcli connection down netplan-enx00e04c68345c 
    sudo nmcli connection up netplan-enx00e04c68345c
```
