import subprocess
import time

def launch_scene(scene_number):
    launch_file = f"load_kuavo_mujoco_sim{scene_number}.launch"
    command = f"roslaunch data_challenge_simulator {launch_file}"
    print(f"启动 {launch_file}...")
    # 使用 Popen 启动场景，异步执行
    subprocess.Popen(command, shell=True)

def launch_task(scene_number):
    task_script = f"task{scene_number}.py"
    command = f"python3 {task_script}"
    print(f"启动 {task_script}...")
    subprocess.run(command, shell=True)

def main():
    print("请选择要启动的任务场景:")
    print("1: 任务1 —— 传送带物品分拣")
    print("2: 任务2 —— 传送带物品称重")
    print("3: 任务3 —— 物品翻面")
    print("4: 任务4 —— 货架物品运送")
    
    # 获取用户输入
    scene_number = input("请输入(1-4): ")
    
    # 检查输入有效性
    if scene_number in ['1', '2', '3', '4']:
        # 启动场景
        launch_scene(scene_number)
        
        # 等待几秒钟以确保场景启动
        time.sleep(4)
        
        # 提示用户是否要启动任务
        task_choice = input("场景启动完成，是否启动任务? (输入 's' 启动任务): ")
        
        if task_choice.lower() == 's':
            launch_task(scene_number)
        else:
            print("未启动任务。")
    else:
        print("无效的输入。请输入 1 到 4 之间的数字。")

if __name__ == "__main__":
    main()
