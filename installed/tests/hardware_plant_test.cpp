#include <vector>
#include "hardware_plant.h"
#include "joint_test_poses.h"
#include <unordered_map>
#include <cstdlib>  // for getenv

#define LEG_JOINT_NUM 12
#define ARM_JOINT_NUM 14
#define HEAD_JOINT_NUM 2
#define JOINT_NUM (LEG_JOINT_NUM+ARM_JOINT_NUM+HEAD_JOINT_NUM)
#define PI 3.14
// #define TO_RADIAN (PI/180)

std::vector<double> init_joints_q(JOINT_NUM, 0);

std::vector<double> test_leg_joints_q;
std::vector<double> test_arm_joints_q;
std::vector<double> test_head_joints_q;

std::unordered_map<std::string, char> end_effector_type_map = {
    {"qiangnao", 'q'},
    {"lejuclaw", 'l'},
    {"qiangnao_touch", 't'}
};

using namespace HighlyDynamic;

void initializeTestPoses() {
    auto test_poses = joint_test_poses::test_pos_list();
    
    test_leg_joints_q = test_poses[0];
    test_arm_joints_q = test_poses[1];
    test_head_joints_q = test_poses[2];
    
    for (int i = 0; i < test_leg_joints_q.size(); i++) {
        test_leg_joints_q[i] = test_leg_joints_q[i];
    }
    for (int i = 0; i < test_arm_joints_q.size(); i++) {
        test_arm_joints_q[i] = test_arm_joints_q[i];
    }
    for (int i = 0; i < test_head_joints_q.size(); i++) {
        test_head_joints_q[i] = test_head_joints_q[i];
    }
}

class HardwareTest
{
public:
    HardwareTest() = default;
    
    ~HardwareTest()
    {
        if (hardware_plant_) {
            hardware_plant_.reset();
        }
    }

    void init(std::string kuavo_assets_path="") {
        const char* robot_version_env = std::getenv("ROBOT_VERSION");
        if (robot_version_env == nullptr) {
            std::cerr << "错误：未设置环境变量 ROBOT_VERSION" << std::endl;
            std::cerr << "使用默认版本 42" << std::endl;
        }
        else{
            this->rb_version = RobotVersion::create(std::atoi(robot_version_env));
            std::cout << "使用环境变量 ROBOT_VERSION: " << this->rb_version.to_string() << std::endl;
        }
        hardware_param = HardwareParam();
        hardware_param.robot_version = this->rb_version;
        if (kuavo_assets_path == ""){
            hardware_param.kuavo_assets_path = KUAVO_ASSETS_PATH; // 使用编译 KUAVO_ASSETS_PATH
        }
        else{
            hardware_param.kuavo_assets_path = kuavo_assets_path;
        }
        std::cout << "kuavo_assets_path: " << hardware_param.kuavo_assets_path << std::endl;
        std::cout << "准备初始化硬件..." << std::endl;
        hardware_plant_ = std::make_unique<HardwarePlant>(dt_, hardware_param, std::string(PROJECT_SOURCE_DIR)); // TODO: 需要兼容预编译的情况
        hardware_plant_->HWPlantInit();
        if (hardware_plant_ == nullptr) {
            std::cout << "硬件初始化失败" << std::endl;
            exit(1);
        }
        else{
            std::cout << "硬件初始化成功" << std::endl;
        }
    }

    void sendJointMoveToRequest(const std::vector<double>& joint_values, const std::string& joint_type) {

        auto output_joint_move_to_request = [](const std::vector<double>& joint_values, const std::string& joint_type){
            std::cout << "发送 " << joint_type << " 关节运动请求" << std::endl;
        };

        output_joint_move_to_request(joint_values, joint_type);

        hardware_plant_->jointMoveTo(joint_values, 30.0, 0.02);

        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        hardware_plant_->jointMoveTo(init_joints_q, 30.0, 0.02);
        
    }

    void testEndEffectors(){
        switch (end_effector_type_map[hardware_plant_->end_effector_type_]){
            case 'q':
                testQiangnao();
                break;
            case 'l':
                testLejuClaw();
                break;
            case 't':
                testQiangnaoTouch();
                break;
            default:
                std::cout << "无效的末端执行器类型: " << hardware_plant_->end_effector_type_ << std::endl;
        }
    }

    void testQiangnaoTouch(){
        std::cout << "测试灵巧手(带触觉反馈)" << std::endl;


        auto output_end_effector_cmd = [](const std::vector<EndEffectorData>& end_effector_cmd){
            std::cout << "控制灵巧手：" << std::endl;
            std::cout << "左手指令：" << std::endl;
            std::cout << "位置：" << end_effector_cmd[0].position.transpose() << std::endl;
            std::cout << "右手指令：" << std::endl;
            std::cout << "位置：" << end_effector_cmd[1].position.transpose() << std::endl;
        };

        auto output_close_finger_status = [](eef_controller::FingerStatusPtrArray& status){
            std::cout << "夹爪状态：" << std::endl;
            std::cout << "左手状态：" << std::endl;
            std::cout << "位置：" ;
            for (int i = 0; i < 6; i++) {
                std::cout << static_cast<int>(status[0]->positions[i]) << " ";
            }
            std::cout << std::endl;
            std::cout << "右手状态：" << std::endl;
            std::cout << "位置：" ;
            for (int i = 0; i < 6; i++) {
                std::cout << static_cast<int>(status[1]->positions[i]) << " ";
            }
            std::cout << std::endl;
        };


        Eigen::VectorXd left_close_position(6);
        left_close_position << 100, 100, 100, 100, 100, 100;
        Eigen::VectorXd right_close_position(6);
        right_close_position << 100, 100, 100, 100, 100, 100;

        std::vector<EndEffectorData> end_effector_cmd;
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao_touch, left_close_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao_touch, right_close_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        output_end_effector_cmd(end_effector_cmd);
        hardware_plant_->endEffectorCommand(end_effector_cmd);
        
        std::this_thread::sleep_for(std::chrono::seconds(3));

        auto close_finger_status = hardware_plant_->getHandControllerStatus();
        output_close_finger_status(close_finger_status);

        Eigen::VectorXd left_open_position(6);
        left_open_position << 0, 0, 0, 0, 0, 0;
        Eigen::VectorXd right_open_position(6);
        right_open_position << 0, 0, 0, 0, 0, 0;

        end_effector_cmd.clear();
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao_touch, left_open_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao_touch, right_open_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        output_end_effector_cmd(end_effector_cmd);
        hardware_plant_->endEffectorCommand(end_effector_cmd);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        auto open_finger_status = hardware_plant_->getHandControllerStatus();
        output_close_finger_status(open_finger_status);
    }

    void testQiangnao()
    {

        auto output_end_effector_cmd = [](const std::vector<EndEffectorData>& end_effector_cmd){
            std::cout << "控制灵巧手：" << std::endl;
            std::cout << "左手指令：" << std::endl;
            std::cout << "位置：" << end_effector_cmd[0].position.transpose() << std::endl;
            std::cout << "右手指令：" << std::endl;
            std::cout << "位置：" << end_effector_cmd[1].position.transpose() << std::endl;
        };

        auto output_close_finger_status = [](eef_controller::FingerStatusPtrArray& status){
            std::cout << "夹爪状态：" << std::endl;
            std::cout << "左手状态：" << std::endl;
            std::cout << "位置：" ;
            for (int i = 0; i < 6; i++) {
                std::cout << static_cast<int>(status[0]->positions[i]) << " ";
            }
            std::cout << std::endl;
            std::cout << "右手状态：" << std::endl;
            std::cout << "位置：" ;
            for (int i = 0; i < 6; i++) {
                std::cout << static_cast<int>(status[1]->positions[i]) << " ";
            }
            std::cout << std::endl;
        };
        Eigen::VectorXd left_close_position(6);
        left_close_position << 100, 100, 100, 100, 100, 100;
        Eigen::VectorXd right_close_position(6);
        right_close_position << 100, 100, 100, 100, 100, 100;

        std::vector<EndEffectorData> end_effector_cmd;
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao, left_close_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao, right_close_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        output_end_effector_cmd(end_effector_cmd);
        hardware_plant_->endEffectorCommand(end_effector_cmd);
        
        std::this_thread::sleep_for(std::chrono::seconds(3));

        auto close_finger_status = hardware_plant_->getHandControllerStatus();
        output_close_finger_status(close_finger_status);

        Eigen::VectorXd left_open_position(6);
        left_open_position << 0, 0, 0, 0, 0, 0;
        Eigen::VectorXd right_open_position(6);
        right_open_position << 0, 0, 0, 0, 0, 0;

        end_effector_cmd.clear();
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao, left_open_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        end_effector_cmd.push_back(EndEffectorData{EndEffectorType::qiangnao, right_open_position, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)});
        output_end_effector_cmd(end_effector_cmd);
        hardware_plant_->endEffectorCommand(end_effector_cmd);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        auto open_finger_status = hardware_plant_->getHandControllerStatus();
        output_close_finger_status(open_finger_status);
        
    }

    void testIMU(){
        std::cout << "测试 IMU 单元" << std::endl;
        const int SAMPLE_COUNT = 100;         // 检测样本数（可根据需求调整）
        const double TIME_INTERVAL = 0.001;     // 采样间隔（秒，可根据IMU频率调整）
        const double MAX_ALLOWED_DRIFT = 0.1; // 允许的最大漂移（弧度，约5.7度）

        std::vector<double> yawAngles; // 存储所有采样的yaw角

        std::cout << "正在检测Yaw角稳定性（保持静止）..." << std::endl;
        Eigen::Vector3d acc, gyro;
        Eigen::Quaterniond quat;
        auto motor_info = hardware_plant_->get_motor_info();
        auto imu_type_str_ = motor_info.getIMUType(rb_version);
        for (int i = 0; i < SAMPLE_COUNT; ++i)
        {
            // 读取最新IMU数据
            bool readflag;
            if (imu_type_str_ == "xsens")
                readflag = xsens_IMU::getImuDataFrame(acc, gyro, quat);
            else
                readflag = HIPNUC_IMU::getImuDataFrame(acc, gyro, quat);
            if (!readflag)
            {
                std::cerr << "检测过程中IMU数据读取失败!" << std::endl;
            }

            // 将四元数转换为Yaw角（ZYX欧拉角顺序，Yaw为绕Z轴的旋转角）
            Eigen::Vector3d eulerAngles = quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
            double yaw = eulerAngles[0];                                                // Yaw角（单位：弧度）

            yawAngles.push_back(yaw);
            std::cout << "采样 " << i + 1 << "/" << SAMPLE_COUNT << "，Yaw: " << yaw << " rad" << std::endl;

            // 等待采样间隔
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(TIME_INTERVAL * 1000)));
        }

        // 计算Yaw角的波动范围（最大值-最小值）
        double maxYaw = *std::max_element(yawAngles.begin(), yawAngles.end());
        double minYaw = *std::min_element(yawAngles.begin(), yawAngles.end());
        double yawDrift = maxYaw - minYaw;

        // 判断是否超过漂移阈值
        if (yawDrift > MAX_ALLOWED_DRIFT)
        {
            std::cerr << "警告：Yaw角漂移检测失败！漂移量：" << yawDrift << " rad（允许最大值：" << MAX_ALLOWED_DRIFT << " rad）" << std::endl;
            std::cerr << "可能原因：IMU未校准、陀螺仪零偏过大或设备晃动" << std::endl;
        }

        std::cout << "Yaw角稳定性检测通过！漂移量：" << yawDrift << " rad" << std::endl;
    }

    void testLejuClaw()
    {
        std::cout << "测试夹爪" << std::endl;
        eef_controller::ControlClawRequest controller_req;
        eef_controller::ControlClawResponse controller_res;
        controller_req.data.name = {"left_claw", "right_claw"};
        controller_req.data.position = {0.0, 0.0};
        controller_req.data.velocity = {50.0, 50.0};
        controller_req.data.effort = {1.0, 1.0};

        auto output_controller_req = [](const eef_controller::ControlClawRequest& controller_req){
            std::cout << "控制夹爪：" << std::endl;
            std::cout << "夹爪名称：" << controller_req.data.name[0] << " " << controller_req.data.name[1] << std::endl;
            std::cout << "夹爪位置：" << controller_req.data.position[0] << " " << controller_req.data.position[1] << std::endl;
            std::cout << "夹爪速度：" << controller_req.data.velocity[0] << " " << controller_req.data.velocity[1] << std::endl;
            std::cout << "夹爪力矩：" << controller_req.data.effort[0] << " " << controller_req.data.effort[1] << std::endl;
        };

        auto output_leju_claw_state = [](const eef_controller::ClawState& state){
            std::cout << "夹爪状态：" << std::endl;
            std::cout << "夹爪位置：" << state.data.position[0] << " " << state.data.position[1] << std::endl;
            std::cout << "夹爪速度：" << state.data.velocity[0] << " " << state.data.velocity[1] << std::endl;
            std::cout << "夹爪力矩：" << state.data.effort[0] << " " << state.data.effort[1] << std::endl;
        };

        auto result = hardware_plant_->controlLejuClaw(controller_req, controller_res);
        output_controller_req(controller_req);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        if (result){
          auto state = hardware_plant_->getLejuClawState();
          output_leju_claw_state(state);
        }
        else{
          std::cout << "夹爪控制失败" << std::endl;
          return;
        }



        controller_req.data.position = {100.0, 100.0};
        result = hardware_plant_->controlLejuClaw(controller_req, controller_res);
        output_controller_req(controller_req);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        if (result){
          auto state = hardware_plant_->getLejuClawState();
          output_leju_claw_state(state);
        }
        else{
          std::cout << "夹爪控制失败" << std::endl;
          return;
        }
    }

private:
    double dt_ = 0.001;
    RobotVersion rb_version{4, 5};
    HardwareParam hardware_param;
    std::unique_ptr<HardwarePlant> hardware_plant_;
};

int main(int argc, char const *argv[])
{
    std::string kuavo_assets_path = "";
    if (argc > 1 && argv[1] != nullptr) {
        kuavo_assets_path = std::string(argv[1]);
    }
    std::cout << "硬件测试程序" << std::endl;
    using namespace HighlyDynamic;
    initializeTestPoses();

    std::cout << "初始化硬件测试" << std::endl;
    auto hardware_test = std::make_shared<HardwareTest>();
    hardware_test->init(kuavo_assets_path);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto output_test_menu = [](){
      std::cout << "[HardwareTest] 按下 'a' 测试腿部电机" << std::endl;
      std::cout << "[HardwareTest] 按下 's' 测试手臂电机" << std::endl;
      std::cout << "[HardwareTest] 按下 'd' 测试头部电机" << std::endl;
      std::cout << "[HardwareTest] 按下 'f' 测试末端执行器" << std::endl;
      std::cout << "[HardawreTest] 按下 'i' 测试 IMU 单元" << std::endl;
      std::cout << "[HardwareTest] 按下 'q' 退出" << std::endl;
    };

    output_test_menu();
    bool running = true;
    while (running)
    {
        char input;
        if (read(STDIN_FILENO, &input, 1) > 0) {
            switch (input) {
                case 'a':
                    hardware_test->sendJointMoveToRequest(test_leg_joints_q, "测试腿部电机");
                    output_test_menu();
                    break;
                case 's':
                    hardware_test->sendJointMoveToRequest(test_arm_joints_q, "测试手臂电机");
                    output_test_menu();
                    break;
                case 'd':
                    hardware_test->sendJointMoveToRequest(test_head_joints_q, "测试头部电机");
                    output_test_menu();
                    break;
                case 'f':
                    hardware_test->testEndEffectors();
                    output_test_menu();
                    break;
                case 'i':
                    hardware_test->testIMU();
                    output_test_menu();
                    break;
                case 'q':
                    std::cout << "[HardwareTest] 退出" << std::endl;
                    hardware_test->sendJointMoveToRequest(init_joints_q, "init");
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    running = false;
                    
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}

