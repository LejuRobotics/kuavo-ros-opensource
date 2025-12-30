# Kuavo Data Challenge Simulator - Installation Guide
## Requirements
- System: Ubuntu 20.04 recommended
- Dependencies: Docker, NVIDIA CUDA Toolkit

## 1. Clone Repository
```bash
git clone https://github.com/LejuRobotics/kuavo-ros-opensource.git
cd kuavo-ros-opensource
git checkout opensource/kuavo-data-challenge/icra
````

## 2. Docker Setup
- Download docker image：
```bash  
wget https://kuavo.lejurobot.com/docker_images/kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz
```

- Load docker image：
```bash
docker load -i kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz
```

- Enter container, The repository will be mounted at /root/kuavo_ws inside the container by default.:
```bash
cd docker
./run_with_gpu.sh # or sudo ./run_with_gpu.sh 
```

## 3. Set the Robot Version Environment Variable

ensure the robot version is 45 — check with: ```echo $ROBOT_VERSION```
```bash
export ROBOT_VERSION=45
```

## 4. Add Keys

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys AD19BAB3CBF125EA
```

## 5. Install Dependencies

```bash
sudo apt-get update
sudo apt-get install ros-noetic-geographic-msgs
```

## 6. Build the Project

```bash
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
source installed/setup.zsh # or source installed/setup.bash
catkin build humanoid_controllers data_challenge_simulator
source devel/setup.zsh
```

## 7. Install the SDK

```bash
cd src/kuavo_humanoid_sdk
./install.sh
```

## 8. Mujoco Simulation Setup

```bash
roslaunch data_challenge_simulator load_kuavo_mujoco_sim1.launch
```

- IMPORTANT NOTES:
  1. If this is your first time launch mujoco, please wait until it finishes loading(you can see info like 'building cppad file 24/27, process: 3/29, please wait for some minutes...' in the terminal.)
  2. The ready signal would be like: the robot stand on the ground without showing 'PAUSE'
  3. If your robot always falls down, this might be due to wbc calculation error. Please go to /data_challenge_simulator/launch/load_kuavo_mujoco_sim{i}.launch(i depends on which scene you launch) and find line 113. There is a parameter ```taskset -c 18``` which indicates the No. of cpu core used for solving wbc. Try modify 18 to 1 or other numbers
  4. You can ctrl c and go to next step when everything is OK. 

- To avoid repeating the environment setup every time, add the following to ~/.zshrc in the kuavo_ws directory:
```bash
export ROBOT_VERSION=45
source devel/setup.zsh
```

# Repository Description
## How to use
### Model Inference

⚠️ Model inference requires both the simulation environment and the inference environment to be running. This document only covers the simulation-side operations for inference. For inference-side details, please refer to the README in the kuavo_data_challenge repositor

After starting inference, you can check logs at kuavo_data_challenge repository's log/kuavo_deploy/deploy.log to diagnose errors.(or simply press l)

enter src/data_challenge_simulator/examples/deploy and run:
```bash
python3 deploy.py
```
Choose the task for this inference run.

The script will automatically open the corresponding scene file and move the robot to the initial pre-grasp pose. At that point, wait for the inference side to respond to begin inference (note: the first run will automatically reset and end all processes).

During inference, if the task completes or the inference ends, the run will automatically terminate all processes and start the next inference run.

Each round's score and the current average score will be displayed in the terminal; only valid data is included in the overall score. Details of scores can also be found at ```data_challenge_simulator/examples/deploy/scores```


