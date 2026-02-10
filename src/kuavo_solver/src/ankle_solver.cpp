#include"kuavo_solver/ankle_solver.h"
#include"kuavo_solver/roban_ankle_solver.h"
#include <memory>

void kuavo_solver::Roban_ankle_solver_deleter::operator()(Roban_ankle_solver* p) const {
    delete p;
}
AnkleSolver::~AnkleSolver() = default;

void AnkleSolver::getconfig(const int ankle_solver_type)
{
    std::cout << "ankle_solver_type: " << ankle_solver_type << std::endl;
    ankle_solver_type_ = ankle_solver_type;
    std::cout << "[AnkleSolver] ankle_solver_type: " << ankle_solver_type_ << std::endl;
    ankle_pitch_limits_ << -1.57, 1.57;
    ankle_roll_limits_ << -1.57, 1.57;
    if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN)
    {
        config.resize(43);
        double z_pitch = -0.416;
        double x_lleq = 0.0385; double y_lleq = 0.03475;  double z_lleq = -0.01;
        double x_lreq = 0.0385; double y_lreq = -0.03475; double z_lreq = -0.01;
        double z_llbar = -0.188;
        double z_lrbar = -0.098;
        double x_lltd = 0.049; double y_lltd = 0.0356172; double z_lltd = -0.00624208;
        double x_lrtd = 0.049; double y_lrtd = -0.035629; double z_lrtd = -0.0061744;
        double l_llbar = 0.03616;
        double l_lrbar = 0.03616;
        double l_lltd = 0.232;
        double l_lrtd = 0.322;

        double x_rleq = 0.0385; double y_rleq = 0.03475; double z_rleq = -0.01;
        double x_rreq = 0.0385; double y_rreq = -0.03475; double z_rreq = -0.01;
        double z_rlbar = -0.098;
        double z_rrbar = -0.188;
        double x_rltd = 0.049; double y_rltd = 0.035629; double z_rltd = -0.0061744;
        double x_rrtd = 0.049; double y_rrtd = -0.0356172; double z_rrtd = -0.00624208;
        double l_rlbar = 0.03616;
        double l_rrbar = 0.03616;
        double l_rltd = 0.322;
        double l_rrtd = 0.232;

        double z_BarKnee = -0.169;
        double l_tendon = 0.16918805677712595;
        double l_BarTd = 0.03500001265742629;
        double l_KneeEq = 0.03500285274088385;
        double qO_knee = 0.26181698840065826;
        double qO_bar = 0.3738812519350086;
        config << z_pitch, 
                  x_lleq, y_lleq, z_lleq, 
                  x_lreq, y_lreq, z_lreq, 
                  z_llbar, z_lrbar, 
                  x_lltd, y_lltd, z_lltd, 
                  x_lrtd, y_lrtd, z_lrtd, 
                  l_llbar, l_lrbar, 
                  l_lltd, l_lrtd, 
                  x_rleq, y_rleq, z_rleq, 
                  x_rreq, y_rreq, z_rreq, 
                  z_rlbar, z_rrbar,
                  x_rltd, y_rltd, z_rltd, 
                  x_rrtd, y_rrtd, z_rrtd, 
                  l_rlbar, l_rrbar,
                  l_rltd, l_rrtd,
                  z_BarKnee, l_tendon, l_BarTd, l_KneeEq, qO_knee, qO_bar;
        ankle_pitch_limits_ << -0.88, 0.35;
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO)
    {
        config.resize(43);
        double z_pitch = -0.346;
        double x_lleq = 0.0385; double y_lleq = 0.0283;  double z_lleq = -0.01;
        double x_lreq = 0.0385; double y_lreq = -0.0283; double z_lreq = -0.01;
        double z_llbar = -0.153;
        double z_lrbar = -0.09;
        double x_lltd = 0.044; double y_lltd = 0.0282564; double z_lltd = -0.0100784;
        double x_lrtd = 0.044; double y_lrtd = -0.0282633; double z_lrtd = -0.0100591;
        double l_llbar = 0.03;
        double l_lrbar = 0.03;
        double l_lltd = 0.193;
        double l_lrtd = 0.256;

        double x_rleq = 0.0385; double y_rleq = 0.0283; double z_rleq = -0.01;
        double x_rreq = 0.0385; double y_rreq = -0.0283; double z_rreq = -0.01;
        double z_rlbar = -0.09;
        double z_rrbar = -0.153;
        double x_rltd = 0.044; double y_rltd = 0.0282633; double z_rltd = -0.0100591;
        double x_rrtd = 0.044; double y_rrtd = -0.0282633; double z_rrtd = -0.0100784;
        double l_rlbar = 0.03;
        double l_rrbar = 0.03;
        double l_rltd = 0.256;
        double l_rrtd = 0.193;

        double z_BarKnee = -0.166;
        double l_tendon = 0.16599763846808185;
        double l_BarTd = 0.03500002038542263;
        double l_KneeEq = 0.03500285274088385;
        double qO_knee = 0.26181698840065826;
        double qO_bar = 0.3986370610067174;
        config << z_pitch, 
                  x_lleq, y_lleq, z_lleq, 
                  x_lreq, y_lreq, z_lreq, 
                  z_llbar, z_lrbar, 
                  x_lltd, y_lltd, z_lltd, 
                  x_lrtd, y_lrtd, z_lrtd, 
                  l_llbar, l_lrbar, 
                  l_lltd, l_lrtd, 
                  x_rleq, y_rleq, z_rleq, 
                  x_rreq, y_rreq, z_rreq, 
                  z_rlbar, z_rrbar,
                  x_rltd, y_rltd, z_rltd, 
                  x_rrtd, y_rrtd, z_rrtd, 
                  l_rlbar, l_rrbar,
                  l_rltd, l_rrtd,
                  z_BarKnee, l_tendon, l_BarTd, l_KneeEq, qO_knee, qO_bar;
        ankle_pitch_limits_ << -0.80, 0.40;

    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN)
    {
        config.resize(37);
        double z_pitch = -0.346;
        double x_lleq = -0.056027; double y_lleq = 0.0255;  double z_lleq = 0.015;
        double x_lreq = -0.056027; double y_lreq = -0.0255; double z_lreq = 0.015;
        double y_llbar = 0.0255;   double z_llbar = -0.09;
        double y_lrbar = -0.0255;  double z_lrbar = -0.153;
        double x_lltd = -0.056027; double z_lltd = 0.015;
        double x_lrtd = -0.056027; double z_lrtd = 0.015;
        double l_lltd = 0.256;
        double l_lrtd = 0.193;
        double l_llbar = 0.058;
        double l_lrbar = 0.058;

        double x_rleq = -0.056027; double y_rleq = 0.0255;  double z_rleq = 0.015;
        double x_rreq = -0.056027; double y_rreq = -0.0255; double z_rreq = 0.015;
        double y_rlbar = 0.0255;   double z_rlbar = -0.153;
        double y_rrbar = -0.0255;  double z_rrbar = -0.09;
        double x_rltd = -0.056027; double z_rltd = 0.015;
        double x_rrtd = -0.056027; double z_rrtd = 0.015;
        double l_rltd = 0.193;
        double l_rrtd = 0.256;
        double l_rlbar = 0.058;
        double l_rrbar = 0.058;
        config << z_pitch, 
                  x_lleq, y_lleq, z_lleq, 
                  x_lreq, y_lreq, z_lreq, 
                  y_llbar, z_llbar, 
                  y_lrbar, z_lrbar, 
                  x_lltd, z_lltd, 
                  x_lrtd, z_lrtd, 
                  l_lltd, l_lrtd, 
                  l_llbar, l_lrbar, 
                  x_rleq, y_rleq, z_rleq, 
                  x_rreq, y_rreq, z_rreq, 
                  y_rlbar, z_rlbar, 
                  y_rrbar, z_rrbar, 
                  x_rltd, z_rltd, 
                  x_rrtd, z_rrtd, 
                  l_rltd, l_rrtd, 
                  l_rlbar, l_rrbar;
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN)
    {
        config.resize(37);
        double z_pitch = -0.235;
        double x_lleq = -0.03919; double y_lleq = 0.028;  double z_lleq = 0.008;
        double x_lreq = -0.03919; double y_lreq = -0.028; double z_lreq = 0.008;
        double y_llbar = 0.028;   double z_llbar = -0.083;
        double y_lrbar = -0.028;  double z_lrbar = -0.138;
        double x_lltd = -0.0391918; double z_lltd = 0.008;
        double x_lrtd = -0.0391918; double z_lrtd = 0.008;
        double l_lltd = 0.15200000001065786;
        double l_lrtd = 0.097000000016701;
        double l_llbar = 0.04;
        double l_lrbar = 0.04;

        double x_rleq = -0.03919; double y_rleq = 0.028;  double z_rleq = 0.008;
        double x_rreq = -0.03919; double y_rreq = -0.028; double z_rreq = 0.008;
        double y_rlbar = 0.028;   double z_rlbar = -0.138;
        double y_rrbar = -0.028;  double z_rrbar = -0.083;
        double x_rltd = -0.0391918; double z_rltd = 0.008;
        double x_rrtd = -0.0391918; double z_rrtd = 0.008;
        double l_rltd = 0.097000000016701;
        double l_rrtd = 0.15200000001065786;
        double l_rlbar = 0.04;
        double l_rrbar = 0.04;
        config << z_pitch, 
                  x_lleq, y_lleq, z_lleq, 
                  x_lreq, y_lreq, z_lreq, 
                  y_llbar, z_llbar, 
                  y_lrbar, z_lrbar, 
                  x_lltd, z_lltd, 
                  x_lrtd, z_lrtd, 
                  l_lltd, l_lrtd, 
                  l_llbar, l_lrbar, 
                  x_rleq, y_rleq, z_rleq, 
                  x_rreq, y_rreq, z_rreq, 
                  y_rlbar, z_rlbar, 
                  y_rrbar, z_rrbar, 
                  x_rltd, z_rltd, 
                  x_rrtd, z_rrtd, 
                  l_rltd, l_rrtd, 
                  l_rlbar, l_rrbar;
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN)
    {
        config.resize(37);
        double z_pitch = -0.235;
        double x_lleq = -0.03919; double y_lleq = 0.03;  double z_lleq = 0.008;
        double x_lreq = -0.03919; double y_lreq = -0.03; double z_lreq = 0.008;
        double y_llbar = 0.03;   double z_llbar = -0.138;
        double y_lrbar = -0.03;  double z_lrbar = -0.083;
        double x_lltd = -0.0391918; double z_lltd = 0.008;
        double x_lrtd = -0.0391918; double z_lrtd = 0.008;
        double l_lltd = 0.097;
        double l_lrtd = 0.152;
        double l_llbar = 0.04;
        double l_lrbar = 0.04;

        double x_rleq = -0.03919; double y_rleq = 0.03;  double z_rleq = 0.008;
        double x_rreq = -0.03919; double y_rreq = -0.03; double z_rreq = 0.008;
        double y_rlbar = 0.03;   double z_rlbar = -0.083;
        double y_rrbar = -0.03;  double z_rrbar = -0.138;
        double x_rltd = -0.0391918; double z_rltd = 0.008;
        double x_rrtd = -0.0391918; double z_rrtd = 0.008;
        double l_rltd = 0.152;
        double l_rrtd = 0.097;
        double l_rlbar = 0.04;
        double l_rrbar = 0.04;
        config << z_pitch, 
                  x_lleq, y_lleq, z_lleq, 
                  x_lreq, y_lreq, z_lreq, 
                  y_llbar, z_llbar, 
                  y_lrbar, z_lrbar, 
                  x_lltd, z_lltd, 
                  x_lrtd, z_lrtd, 
                  l_lltd, l_lrtd, 
                  l_llbar, l_lrbar, 
                  x_rleq, y_rleq, z_rleq, 
                  x_rreq, y_rreq, z_rreq, 
                  y_rlbar, z_rlbar, 
                  y_rrbar, z_rrbar, 
                  x_rltd, z_rltd, 
                  x_rrtd, z_rrtd, 
                  l_rltd, l_rrtd, 
                  l_rlbar, l_rrbar;
        ankle_pitch_limits_ << -0.872664625997165, 0.523598775598299;
        ankle_roll_limits_ << -0.261799387799149, 0.261799387799149;
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2)
    {
        // S2GEN_2 
        // 右腿: rl=长肌腱(内侧y>0), rr=短肌腱(外侧y<0)
        // 左腿: lr=长肌腱(内侧y<0), ll=短肌腱(外侧y>0) 
        config.resize(37);
        
        // 硬编码参数 
        // 关节偏移参数
        double z_pitch = -0.294;      // foot_roll 相对于 knee 的 z 偏移 (-0.276 - 0.018)
        double z_roll = -0.018;       // roll 关节相对于 pitch 关节的 z 偏移
        double x_pitch = 0.03;        // pitch 关节相对于 knee 的 x 偏移
        
        // ===== 左脚踝参数  =====
        // ll = 短肌腱 (外侧, y>0)
        double x_lleq = 0.02780675; double y_lleq = 0.0207;  double z_lleq = 0.00367032;
        double x_llbar = 0.047123;    double z_llbar = -0.1595;   // 短肌腱侧 bar (rr镜像)
        double x_lltd = 0.014; double y_lltd = 0.023428; double z_lltd = -0.0039472;   // 短肌腱附着点 (rr镜像)
        double l0_ll_eqtd = 0.12695512289393013;  // 短肌腱初始长度
        double l_llbar = 0.02375818957412370;     // 短肌腱侧 bar 长度
        
        // lr = 长肌腱 (内侧, y<0)
        double x_lreq = 0.02780675; double y_lreq = -0.0186; double z_lreq = 0.00367032;
        double x_lrbar = 0.05117;     double z_lrbar = -0.10163;  // 长肌腱侧 bar (rl镜像)
        double x_lrtd = 0.014; double y_lrtd = -0.023357; double z_lrtd = -0.0044851;  // 长肌腱附着点 (rl镜像)
        double l0_lr_eqtd = 0.18442304081686461;  // 长肌腱初始长度
        double l_lrbar = 0.02378372491873382;     // 长肌腱侧 bar 长度

        // ===== 右脚踝参数  =====
        // rl = 内侧长肌腱 (y > 0)
        double x_rleq = 0.02780675; double y_rleq = 0.0186;  double z_rleq = 0.00367032;  
        double x_rlbar = 0.05117;     double z_rlbar = -0.10163;  // 长肌腱侧 bar
        double x_rltd = 0.014; double y_rltd = 0.023357; double z_rltd = -0.0044851;
        double l0_rl_eqtd = 0.18442304081686461;  // 长肌腱初始长度
        double l_rlbar = 0.02378372491873382;     // 长肌腱侧 bar 长度
        
        // rr = 外侧短肌腱 (y < 0)
        double x_rreq = 0.02780675; double y_rreq = -0.0207; double z_rreq = 0.00367032;  // 与 XML r_r_eq 一致
        double x_rrbar = 0.047123;    double z_rrbar = -0.1595;   // 短肌腱侧 bar
        double x_rrtd = 0.014; double y_rrtd = -0.023428; double z_rrtd = -0.0039472;
        double l0_rr_eqtd = 0.12695512289393013;  // 短肌腱初始长度
        double l_rrbar = 0.02375818957412370;     // 短肌腱侧 bar 长度

        config << z_pitch, 
                  x_lleq, y_lleq, z_lleq, 
                  x_lreq, y_lreq, z_lreq, 
                  x_llbar, z_llbar,       // 更新为 x_bar
                  x_lrbar, z_lrbar,       // 更新为 x_bar
                  x_lltd, z_lltd, 
                  x_lrtd, z_lrtd, 
                  l0_ll_eqtd, l0_lr_eqtd, 
                  l_llbar, l_lrbar, 
                  x_rleq, y_rleq, z_rleq, 
                  x_rreq, y_rreq, z_rreq, 
                  x_rlbar, z_rlbar,       // 更新为 x_bar
                  x_rrbar, z_rrbar,       // 更新为 x_bar
                  x_rltd, z_rltd, 
                  x_rrtd, z_rrtd, 
                  l0_rl_eqtd, l0_rr_eqtd, 
                  l_rlbar, l_rrbar;
        // 左脚pitch限位
        ankle_pitch_limits_ << -0.872664625997165, 0.523598775598299;
        ankle_roll_limits_ << -0.261799387799149, 0.261799387799149;
        
        // 初始化 roban_ankle_solver（使用自定义删除器）
        kuavo_solver::RobanAnkleParams params;
        // 关节偏移参数
        params.z_pitch = z_pitch;
        params.z_roll = z_roll;
        params.x_pitch = x_pitch;
        // 左脚踝参数
        params.x_lleq = x_lleq; params.y_lleq = y_lleq; params.z_lleq = z_lleq;
        params.x_lreq = x_lreq; params.y_lreq = y_lreq; params.z_lreq = z_lreq;
        params.x_llbar = x_llbar; params.z_llbar = z_llbar;
        params.x_lrbar = x_lrbar; params.z_lrbar = z_lrbar;
        params.x_lltd = x_lltd; params.y_lltd = y_lltd; params.z_lltd = z_lltd;
        params.x_lrtd = x_lrtd; params.y_lrtd = y_lrtd; params.z_lrtd = z_lrtd;
        params.l0_ll_eqtd = l0_ll_eqtd; params.l0_lr_eqtd = l0_lr_eqtd;
        // 右脚踝参数
        params.x_rleq = x_rleq; params.y_rleq = y_rleq; params.z_rleq = z_rleq;
        params.x_rreq = x_rreq; params.y_rreq = y_rreq; params.z_rreq = z_rreq;
        params.x_rlbar = x_rlbar; params.z_rlbar = z_rlbar;
        params.x_rrbar = x_rrbar; params.z_rrbar = z_rrbar;
        params.x_rltd = x_rltd; params.y_rltd = y_rltd; params.z_rltd = z_rltd;
        params.x_rrtd = x_rrtd; params.y_rrtd = y_rrtd; params.z_rrtd = z_rrtd;
        params.l0_rl_eqtd = l0_rl_eqtd; params.l0_rr_eqtd = l0_rr_eqtd;
        // 求解器参数
        params.default_tolerance = 1e-8;
        params.max_iterations = 10;
        
        roban_solver_ = std::unique_ptr<kuavo_solver::Roban_ankle_solver, kuavo_solver::Roban_ankle_solver_deleter>(
            new kuavo_solver::Roban_ankle_solver(params));
    }
}

void AnkleSolver::applyRollLimitBasedOnPitch(Eigen::VectorXd& joint_q)
{
    // 根据 pitch 值动态计算 roll 的限制
    // 线性插值：pitch=0 时 roll_limit=1.0, pitch=0.4 时 roll_limit=0.1
    // pitch<0.0 时无限制
    auto calculateRollLimit = [](double pitch) -> double {
        if (pitch > 0.4) {
            // pitch > 0.4 时无限制，返回一个很大的值
            return 0.1;  // 足够大的值，表示无限制
        } else if (pitch <= 0.0) {
            // pitch <= 0 时，roll_limit = 1.0
            return 1.57;
        } else {
            // 0 < pitch <= 0.4 时，线性插值
            // roll_limit = 1.0 - 2.25 * pitch
            // 斜率：2.59，根据实际调整避开不可达的工作空间
            double roll_limit = 1.0 - 2.65 * pitch;
            return std::max(0.0, roll_limit);  // 确保不小于 0.1
        }
    };
    
    // 左腿：根据 joint_q[4] (pitch) 限制 joint_q[5] (roll)
    double left_pitch = joint_q[4];
    double left_roll_limit = calculateRollLimit(left_pitch);
    joint_q[5] = std::max(std::min(joint_q[5], left_roll_limit), -left_roll_limit);
    
    // 右腿：根据 joint_q[10] (pitch) 限制 joint_q[11] (roll)
    double right_pitch = joint_q[10];
    double right_roll_limit = calculateRollLimit(right_pitch);
    joint_q[11] = std::max(std::min(joint_q[11], right_roll_limit), -right_roll_limit);
}

Eigen::VectorXd AnkleSolver::joint_to_motor_position(const Eigen::VectorXd& q)
{
    Eigen::VectorXd result(12);
    auto joint_q = q;
    // 膝关节输入限制：膝关节位置必须 >= 0，小于 0 进行截断到 0
    // joint_q 索引约定：左膝 joint_q[3]，右膝 joint_q[9]
    joint_q[3] = std::max(joint_q[3], 0.0);
    joint_q[9] = std::max(joint_q[9], 0.0);
    joint_q[4] = std::max(std::min(joint_q[4], ankle_pitch_limits_[1]), ankle_pitch_limits_[0]);
    joint_q[10] = std::max(std::min(joint_q[10], ankle_pitch_limits_[1]), ankle_pitch_limits_[0]);
    joint_q[5] = std::max(std::min(joint_q[5], ankle_roll_limits_[1]), ankle_roll_limits_[0]);
    joint_q[11] = std::max(std::min(joint_q[11], ankle_roll_limits_[1]), ankle_roll_limits_[0]);
    
    if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN)
    {
        result = joint_to_motor_position_pro_(joint_q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO)
    {
        applyRollLimitBasedOnPitch(joint_q);
        result = joint_to_motor_position_pro_(joint_q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN)
    {
        result = joint_to_motor_position_(joint_q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN)
    {
        result = joint_to_motor_position_(joint_q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN)
    {
        result = joint_to_motor_position_s2_(q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2)
    {
        result = joint_to_motor_position_s2_2_(joint_q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_NONE)
    {
        result = joint_q;
    }
    return result;
}
Eigen::VectorXd AnkleSolver::joint_to_motor_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp)
{
    Eigen::VectorXd result(12);
    if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN)
    {
        result = joint_to_motor_velocity_pro_(q, p, dp);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO)
    {
        result = joint_to_motor_velocity_pro_(q, p, dp);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN)
    {
        result = joint_to_motor_velocity_(q, p, dp);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN)
    {
        result = joint_to_motor_velocity_(q, p, dp);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN)
    {
        result = joint_to_motor_velocity_s2_(q, p, dp);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2)
    {
        result = joint_to_motor_velocity_s2_2_(q, p, dp);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_NONE)
    {
        result = dp;
    }
    return result;
}
Eigen::VectorXd AnkleSolver::joint_to_motor_current(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t)
{
    Eigen::VectorXd result(12);
    if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN)
    {
        result = joint_to_motor_current_pro_(q, p, t);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO)
    {
        result = joint_to_motor_current_pro_(q, p, t);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN)
    {
        result = joint_to_motor_current_(q, p, t);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN)
    {
        result = joint_to_motor_current_(q, p, t);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN)
    {
        result = joint_to_motor_current_s2_(q, p, t);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2)
    {
        result = joint_to_motor_current_s2_2_(q, p, t);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_NONE)
    {
        result = t;
    }
    return result;
}
Eigen::VectorXd AnkleSolver::motor_to_joint_position(const Eigen::VectorXd& q)
{
    Eigen::VectorXd result(12);
    if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN)
    {
        result = motor_to_joint_position_pro_(q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO)
    {
        result = motor_to_joint_position_pro_(q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN)
    {
        result = motor_to_joint_position_(q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN)
    {
        result = motor_to_joint_position_(q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN)
    {
        result = motor_to_joint_position_s2_(q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2)
    {
        result = motor_to_joint_position_s2_2_(q);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_NONE)
    {
        result = q;
    }
    return result;
}
Eigen::VectorXd AnkleSolver::motor_to_joint_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v)
{
    Eigen::VectorXd result(12);
    if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN)
    {
        result = motor_to_joint_velocity_pro_(q, p, v);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO)
    {
        result = motor_to_joint_velocity_pro_(q, p, v);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN)
    {
        result = motor_to_joint_velocity_(q, p, v);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN)
    {
        result = motor_to_joint_velocity_(q, p, v);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN)
    {
        result = motor_to_joint_velocity_s2_(q, p, v);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2)
    {
        result = motor_to_joint_velocity_s2_2_(q, p, v);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_NONE)
    {
        result = v;
    }
    return result;
}
Eigen::VectorXd AnkleSolver::motor_to_joint_torque(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c)
{
    Eigen::VectorXd result(12);
    if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN)
    {
        result = motor_to_joint_torque_pro_(q, p, c);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO)
    {
        result = motor_to_joint_torque_pro_(q, p, c);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN)
    {
        result = motor_to_joint_torque_(q, p, c);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S1GEN)
    {
        result = motor_to_joint_torque_(q, p, c);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN)
    {
        result = motor_to_joint_torque_s2_(q, p, c);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2)
    {
        result = motor_to_joint_torque_s2_2_(q, p, c);
    }
    else if (ankle_solver_type_ == AnkleSolverType::ANKLE_SOLVER_TYPE_NONE)
    {
        result = c;
    }
    return result;
}   


/**
 * @brief foot_t => motor_c
 *
 * @param q leg joint position
 * @param p motor joint position
 * @param t leg joint torque
 */
Eigen::VectorXd AnkleSolver::joint_to_motor_current_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t) 
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[4], p6 = p[5];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[10], p12 = p[11];
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];
    double t1 = t[0], t2 = t[1], t3 = t[2], t4 = t[3], t5 = t[4], t6 = t[5];
    double t7 = t[6], t8 = t[7], t9 = t[8], t10 = t[9], t11 = t[10], t12 = t[11];

    // left
    double iq1 = t1;
    double iq2 = t2;
    double iq3 = t3;
    double iq4 = t4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                0., -y_lleq * sq6 - z_lleq * cq6,
                -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                0., -y_lreq * sq6 - z_lreq * cq6,
                -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << -x_lltd * sp5 + z_lltd * cp5, 0,
                0, 0,
                -x_lltd * cp5 - z_lltd * sp5, 0,
                0, -x_lrtd * sp6 + z_lrtd * cp6,
                0, 0,
                0, -x_lrtd * cp6 - z_lrtd * sp6;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5,
                -y_llbar + y_lleq * cq6 - z_lleq * sq6,
                -x_lleq * sq5 + x_lltd * sp5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp5 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6,
                -y_lrbar + y_lreq * cq6 - z_lreq * sq6,
                -x_lreq * sq5 + x_lrtd * sp6 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp6 + z_pitch;

    Eigen::MatrixXd JAct_Ak = (Jxx_l_Td * JAk_p_WEq).inverse() * (Jxx_l_Td * JAct_p_WTd);

    Eigen::Vector2d tau_Ak;
    tau_Ak << t5, t6;

    Eigen::Vector2d tau_Act = JAct_Ak.transpose() * tau_Ak;
    double iq5 = tau_Act[0];
    double iq6 = tau_Act[1];

    // right
    double iq7 = t7;
    double iq8 = t8;
    double iq9 = t9;
    double iq10 = t10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                    0, -y_rleq * sq12 - z_rleq * cq12,
                    -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                    -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                    0, -y_rreq * sq12 - z_rreq * cq12,
                    -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << -x_rltd * sp12 + z_rltd * cp12, 0,
                        0, 0,
                        -x_rltd * cp12 - z_rltd * sp12, 0,
                        0, -x_rrtd * sp11 + z_rrtd * cp11,
                        0, 0,
                        0, -x_rrtd * cp11 - z_rrtd * sp11;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12,
                    -y_rlbar + y_rleq * cq12 - z_rleq * sq12,
                    -x_rleq * sq11 + x_rltd * sp12 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp12 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11,
                    -y_rrbar + y_rreq * cq12 - z_rreq * sq12,
                    -x_rreq * sq11 + x_rrtd * sp11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp11 + z_pitch;

    Eigen::MatrixXd JAct_Ak_right = (Jxx_l_Td_right * JAk_p_WEq_right).inverse() * (Jxx_l_Td_right * JAct_p_WTd_right);

    Eigen::Vector2d tau_Ak_right;
    tau_Ak_right << t11, t12;

    Eigen::Vector2d tau_Act_right = JAct_Ak_right.transpose() * tau_Ak_right;
    double iq12 = tau_Act_right[0];
    double iq11 = tau_Act_right[1];

    Eigen::VectorXd result(12);
    result << iq1, iq2, iq3, iq4, iq5, iq6, iq7, iq8, iq9, iq10, iq11, iq12;
    return result;
}
/**
 * @brief foot_v => motor_v
 *
 * @param q leg joint position
 * @param p motor joint position
 * @param v leg joint velocity
 */
Eigen::VectorXd AnkleSolver::joint_to_motor_velocity_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp) 
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[4], p6 = p[5];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[10], p12 = p[11];
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];
    double dq1 = dp[0], dq2 = dp[1], dq3 = dp[2], dq4 = dp[3], dq5 = dp[4], dq6 = dp[5];
    double dq7 = dp[6], dq8 = dp[7], dq9 = dp[8], dq10 = dp[9], dq11 = dp[10], dq12 = dp[11];

    // left
    double v1 = dq1;
    double v2 = dq2;
    double v3 = dq3;
    double v4 = dq4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                0., -y_lleq * sq6 - z_lleq * cq6,
                -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                0., -y_lreq * sq6 - z_lreq * cq6,
                -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << -x_lltd * sp5 + z_lltd * cp5, 0,
                0, 0,
                -x_lltd * cp5 - z_lltd * sp5, 0,
                0, -x_lrtd * sp6 + z_lrtd * cp6,
                0, 0,
                0, -x_lrtd * cp6 - z_lrtd * sp6;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5,
                -y_llbar + y_lleq * cq6 - z_lleq * sq6,
                -x_lleq * sq5 + x_lltd * sp5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp5 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6,
                -y_lrbar + y_lreq * cq6 - z_lreq * sq6,
                -x_lreq * sq5 + x_lrtd * sp6 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp6 + z_pitch;

    Eigen::Vector2d AkDt;
    AkDt << dq5, dq6;

    Eigen::Vector2d ActDt = (Jxx_l_Td * JAct_p_WTd).colPivHouseholderQr().solve(Jxx_l_Td * JAk_p_WEq * AkDt);
    double v5 = ActDt[0];
    double v6 = ActDt[1];

    // right
    double v7 = dq7;
    double v8 = dq8;
    double v9 = dq9;
    double v10 = dq10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                    0, -y_rleq * sq12 - z_rleq * cq12,
                    -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                    -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                    0, -y_rreq * sq12 - z_rreq * cq12,
                    -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << -x_rltd * sp12 + z_rltd * cp12, 0,
                        0, 0,
                        -x_rltd * cp12 - z_rltd * sp12, 0,
                        0, -x_rrtd * sp11 + z_rrtd * cp11,
                        0, 0,
                        0, -x_rrtd * cp11 - z_rrtd * sp11;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12,
                    -y_rlbar + y_rleq * cq12 - z_rleq * sq12,
                    -x_rleq * sq11 + x_rltd * sp12 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp12 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11,
                    -y_rrbar + y_rreq * cq12 - z_rreq * sq12,
                    -x_rreq * sq11 + x_rrtd * sp11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp11 + z_pitch;

    Eigen::Vector2d AkDt_right;
    AkDt_right << dq11, dq12;

    Eigen::Vector2d ActDt_right = (Jxx_l_Td_right * JAct_p_WTd_right).colPivHouseholderQr().solve(Jxx_l_Td_right * JAk_p_WEq_right * AkDt_right);
    double v12 = ActDt_right[0];
    double v11 = ActDt_right[1];

    Eigen::VectorXd result(12);
    result << v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
    return result;
}
/**
 * @brief foot_q => motor_p
 *
 * @param q leg joint position
 */
Eigen::VectorXd AnkleSolver::joint_to_motor_position_(const Eigen::VectorXd& q)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];

    // left
    double p1 = q1;
    double p2 = q2;
    double p3 = q3;
    double p4 = q4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);

    double x_LlbarEq_W = x_lleq * cq5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6;
    double z_LlbarEq_W = -x_lleq * sq5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 + z_pitch;
    double x_LrbarEq_W = x_lreq * cq5 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6;
    double z_LrbarEq_W = -x_lreq * sq5 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 + z_pitch;
    double y_LltdEq_W = -y_llbar + y_lleq * cq6 - z_lleq * sq6;
    double y_LrtdEq_W = -y_lrbar + y_lreq * cq6 - z_lreq * sq6;
    double b_ll = std::sqrt(x_LlbarEq_W * x_LlbarEq_W + z_LlbarEq_W * z_LlbarEq_W);
    double a_ll = l_llbar;
    double c_ll = std::sqrt(l_lltd * l_lltd - y_LltdEq_W * y_LltdEq_W);
    double p5 = std::atan2(z_LlbarEq_W, -x_LlbarEq_W) + std::acos((a_ll * a_ll + b_ll * b_ll - c_ll * c_ll) / (2 * a_ll * b_ll)) - std::atan2(z_lltd, -x_lltd);
    double b_lr = std::sqrt(x_LrbarEq_W * x_LrbarEq_W + z_LrbarEq_W * z_LrbarEq_W);
    double a_lr = l_lrbar;
    double c_lr = std::sqrt(l_lrtd * l_lrtd - y_LrtdEq_W * y_LrtdEq_W);
    double p6 = std::atan2(z_LrbarEq_W, -x_LrbarEq_W) + std::acos((a_lr * a_lr + b_lr * b_lr - c_lr * c_lr) / (2 * a_lr * b_lr)) - std::atan2(z_lrtd, -x_lrtd);

    // right
    double p7 = q7;
    double p8 = q8;
    double p9 = q9;
    double p10 = q10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);

    x_LlbarEq_W = x_rleq * cq11 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12;
    z_LlbarEq_W = -x_rleq * sq11 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 + z_pitch;
    x_LrbarEq_W = x_rreq * cq11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12;
    z_LrbarEq_W = -x_rreq * sq11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 + z_pitch;
    y_LltdEq_W = -y_rlbar + y_rleq * cq12 - z_rleq * sq12;
    y_LrtdEq_W = -y_rrbar + y_rreq * cq12 - z_rreq * sq12;
    double b_rl = std::sqrt(x_LlbarEq_W * x_LlbarEq_W + z_LlbarEq_W * z_LlbarEq_W);
    double a_rl = l_rlbar;
    double c_rl = std::sqrt(l_rltd * l_rltd - y_LltdEq_W * y_LltdEq_W);
    double p12 = std::atan2(z_LlbarEq_W, -x_LlbarEq_W) + std::acos((a_rl * a_rl + b_rl * b_rl - c_rl * c_rl) / (2 * a_rl * b_rl)) - std::atan2(z_rltd, -x_rltd);
    double b_rr = std::sqrt(x_LrbarEq_W * x_LrbarEq_W + z_LrbarEq_W * z_LrbarEq_W);
    double a_rr = l_rrbar;
    double c_rr = std::sqrt(l_rrtd * l_rrtd - y_LrtdEq_W * y_LrtdEq_W);
    double p11 = std::atan2(z_LrbarEq_W, -x_LrbarEq_W) + std::acos((a_rr * a_rr + b_rr * b_rr - c_rr * c_rr) / (2 * a_rr * b_rr)) - std::atan2(z_rrtd, -x_rrtd);

    Eigen::VectorXd result(12);
    result << p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12;
    return result;
}

/**
 * @brief motor_c => foot_t
 *
 * @param q leg joint position
 * @param p leg motor position
 * @param c leg motor current
 */
Eigen::VectorXd AnkleSolver::motor_to_joint_torque_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[4], p6 = p[5];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[10], p12 = p[11];
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];
    double i1 = c[0], i2 = c[1], i3 = c[2], i4 = c[3], i5 = c[4], i6 = c[5];
    double i7 = c[6], i8 = c[7], i9 = c[8], i10 = c[9], i11 = c[10], i12 = c[11];

    // left
    double t1 = i1;
    double t2 = i2;
    double t3 = i3;
    double t4 = i4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                0., -y_lleq * sq6 - z_lleq * cq6,
                -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                0., -y_lreq * sq6 - z_lreq * cq6,
                -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << -x_lltd * sp5 + z_lltd * cp5, 0,
                0, 0,
                -x_lltd * cp5 - z_lltd * sp5, 0,
                0, -x_lrtd * sp6 + z_lrtd * cp6,
                0, 0,
                0, -x_lrtd * cp6 - z_lrtd * sp6;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5,
                -y_llbar + y_lleq * cq6 - z_lleq * sq6,
                -x_lleq * sq5 + x_lltd * sp5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp5 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6,
                -y_lrbar + y_lreq * cq6 - z_lreq * sq6,
                -x_lreq * sq5 + x_lrtd * sp6 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp6 + z_pitch;

    Eigen::MatrixXd JAk_Act = (Jxx_l_Td * JAct_p_WTd).inverse() * (Jxx_l_Td * JAk_p_WEq);

    Eigen::Vector2d tau_Act;
    tau_Act << i5, i6;

    Eigen::Vector2d tau_Ak = JAk_Act.transpose() * tau_Act;
    double t5 = tau_Ak[0];
    double t6 = tau_Ak[1];

    // right
    double t7 = i7;
    double t8 = i8;
    double t9 = i9;
    double t10 = i10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                    0, -y_rleq * sq12 - z_rleq * cq12,
                    -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                    -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                    0, -y_rreq * sq12 - z_rreq * cq12,
                    -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << -x_rltd * sp12 + z_rltd * cp12, 0,
                        0, 0,
                        -x_rltd * cp12 - z_rltd * sp12, 0,
                        0, -x_rrtd * sp11 + z_rrtd * cp11,
                        0, 0,
                        0, -x_rrtd * cp11 - z_rrtd * sp11;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12,
                    -y_rlbar + y_rleq * cq12 - z_rleq * sq12,
                    -x_rleq * sq11 + x_rltd * sp12 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp12 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11,
                    -y_rrbar + y_rreq * cq12 - z_rreq * sq12,
                    -x_rreq * sq11 + x_rrtd * sp11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp11 + z_pitch;

    Eigen::MatrixXd JAk_Act_right = (Jxx_l_Td_right * JAct_p_WTd_right).inverse() * (Jxx_l_Td_right * JAk_p_WEq_right);

    Eigen::Vector2d tau_Act_right;
    tau_Act_right << i12, i11;

    Eigen::Vector2d tau_Ak_right = JAk_Act_right.transpose() * tau_Act_right;
    double t11 = tau_Ak_right[0];
    double t12 = tau_Ak_right[1];

    Eigen::VectorXd result(12);
    result << t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
    return result;
}
/**
 * @brief motor_v => foot_v
 *
 * @param q leg joint position
 * @param p leg motor position
 * @param v leg motor velocity
 */
Eigen::VectorXd AnkleSolver::motor_to_joint_velocity_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[4], p6 = p[5];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[10], p12 = p[11];
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];
    double v1 = v[0], v2 = v[1], v3 = v[2], v4 = v[3], v5 = v[4], v6 = v[5];
    double v7 = v[6], v8 = v[7], v9 = v[8], v10 = v[9], v11 = v[10], v12 = v[11];

    // left
    double dq1 = v1;
    double dq2 = v2;
    double dq3 = v3;
    double dq4 = v4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                0., -y_lleq * sq6 - z_lleq * cq6,
                -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                0., -y_lreq * sq6 - z_lreq * cq6,
                -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << -x_lltd * sp5 + z_lltd * cp5, 0,
                0, 0,
                -x_lltd * cp5 - z_lltd * sp5, 0,
                0, -x_lrtd * sp6 + z_lrtd * cp6,
                0, 0,
                0, -x_lrtd * cp6 - z_lrtd * sp6;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5,
                -y_llbar + y_lleq * cq6 - z_lleq * sq6,
                -x_lleq * sq5 + x_lltd * sp5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp5 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6,
                -y_lrbar + y_lreq * cq6 - z_lreq * sq6,
                -x_lreq * sq5 + x_lrtd * sp6 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp6 + z_pitch;

    Eigen::Vector2d ActDt;
    ActDt << v5, v6;

    Eigen::Vector2d AkDt = (Jxx_l_Td * JAk_p_WEq).colPivHouseholderQr().solve(Jxx_l_Td * JAct_p_WTd * ActDt);
    double dq5 = AkDt[0];
    double dq6 = AkDt[1];

    // right
    double dq7 = v7;
    double dq8 = v8;
    double dq9 = v9;
    double dq10 = v10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                    0, -y_rleq * sq12 - z_rleq * cq12,
                    -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                    -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                    0, -y_rreq * sq12 - z_rreq * cq12,
                    -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << -x_rltd * sp12 + z_rltd * cp12, 0,
                        0, 0,
                        -x_rltd * cp12 - z_rltd * sp12, 0,
                        0, -x_rrtd * sp11 + z_rrtd * cp11,
                        0, 0,
                        0, -x_rrtd * cp11 - z_rrtd * sp11;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12,
                    -y_rlbar + y_rleq * cq12 - z_rleq * sq12,
                    -x_rleq * sq11 + x_rltd * sp12 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp12 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11,
                    -y_rrbar + y_rreq * cq12 - z_rreq * sq12,
                    -x_rreq * sq11 + x_rrtd * sp11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp11 + z_pitch;

    Eigen::Vector2d ActDt_right;
    ActDt_right << v12, v11;

    Eigen::Vector2d AkDt_right = (Jxx_l_Td_right * JAk_p_WEq_right).colPivHouseholderQr().solve(Jxx_l_Td_right * JAct_p_WTd_right * ActDt_right);
    double dq11 = AkDt_right[0];
    double dq12 = AkDt_right[1];

    Eigen::VectorXd result(12);
    result << dq1, dq2, dq3, dq4, dq5, dq6, dq7, dq8, dq9, dq10, dq11, dq12;
    return result;
}
/**
 * @brief motor_q => foot_q
 *
 * @param p leg motor position
 */
Eigen::VectorXd AnkleSolver::motor_to_joint_position_(const Eigen::VectorXd& p) 
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[4], p6 = p[5];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[10], p12 = p[11];

    // left
    double q1 = p1;
    double q2 = p2;
    double q3 = p3;
    double q4 = p4;

    double q5 = 0.0, q6 = 0.0;
    for (int i = 0; i < N_ITER; ++i) {
        double q50 = q5, q60 = q6;
        double cq5 = std::cos(q50), sq5 = std::sin(q50);
        double cq6 = std::cos(q60), sq6 = std::sin(q60);
        double cp5 = std::cos(p5), sp5 = std::sin(p5);
        double cp6 = std::cos(p6), sp6 = std::sin(p6);

        double x_LltendonEq_W = x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5;
        double y_LltendonEq_W = -y_llbar + y_lleq * cq6 - z_lleq * sq6;
        double z_LltendonEq_W = -std::sqrt(l_lltd * l_lltd - x_LltendonEq_W * x_LltendonEq_W - y_LltendonEq_W * y_LltendonEq_W);
        double z_WLltendon = -x_lltd * sp5 + z_llbar + z_lltd * cp5;
        double z_PitchLleq_W = z_LltendonEq_W - (z_pitch - z_WLltendon);
        double x_LrtendonEq_W = x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6;
        double y_LrtendonEq_W = -y_lrbar + y_lreq * cq6 - z_lreq * sq6;
        double z_LrtendonEq_W = -std::sqrt(l_lrtd * l_lrtd - x_LrtendonEq_W * x_LrtendonEq_W - y_LrtendonEq_W * y_LrtendonEq_W);
        double z_WLrtendon = -x_lrtd * sp6 + z_lrbar + z_lrtd * cp6;
        double z_PitchLreq_W = z_LrtendonEq_W - (z_pitch - z_WLrtendon);
        double z_PitchLtoe_W = (z_PitchLleq_W + z_PitchLreq_W) / 2.0;

        double x = x_lleq;
        double y = y_lleq;
        double z = z_lleq;
        double avg = z_PitchLtoe_W;
        double dev = (z_PitchLleq_W - z_PitchLreq_W) / 2.0;
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        double avg2 = avg * avg;
        double dev2 = dev * dev;
        double delta = avg2 * x2 - (x2 + z2) * (avg2 + z2 * dev2 / y2 - z2);
        if (delta < 0.0) {
            delta = 0.0;
        }
        double s_1 = (-avg * x - std::sqrt(delta)) / (x2 + z2);
        q5 = std::asin(s_1);
        double z_LrLleq_W = z_PitchLleq_W - z_PitchLreq_W;
        double z_LrLleq_Pitch = z_LrLleq_W / std::cos(q5);
        q6 = std::asin(z_LrLleq_Pitch / (y_lleq - y_lreq));
        if ((std::abs(q5 - q50) + std::abs(q6 - q60)) < 1e-6) {
            break;
        }
    }

    // right
    double q7 = p7;
    double q8 = p8;
    double q9 = p9;
    double q10 = p10;

    double q11 = 0.0, q12 = 0.0;
    for (int i = 0; i < N_ITER; ++i) {
        double q110 = q11, q120 = q12;
        double cq11 = std::cos(q110), sq11 = std::sin(q110);
        double cq12 = std::cos(q120), sq12 = std::sin(q120);
        double cp12 = std::cos(p12), sp12 = std::sin(p12);
        double cp11 = std::cos(p11), sp11 = std::sin(p11);

        double x_RltendonEq_W = x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12;
        double y_RltendonEq_W = -y_rlbar + y_rleq * cq12 - z_rleq * sq12;
        double z_RltendonEq_W = -std::sqrt(l_rltd * l_rltd - x_RltendonEq_W * x_RltendonEq_W - y_RltendonEq_W * y_RltendonEq_W);
        double z_WRltendon = -x_rltd * sp12 + z_rlbar + z_rltd * cp12;
        double z_PitchRleq_W = z_RltendonEq_W - (z_pitch - z_WRltendon);
        double x_RrtendonEq_W = x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11;
        double y_RrtendonEq_W = -y_rrbar + y_rreq * cq12 - z_rreq * sq12;
        double z_RrtendonEq_W = -std::sqrt(l_rrtd * l_rrtd - x_RrtendonEq_W * x_RrtendonEq_W - y_RrtendonEq_W * y_RrtendonEq_W);
        double z_WRrtendon = -x_rrtd * sp11 + z_rrbar + z_rrtd * cp11;
        double z_PitchRreq_W = z_RrtendonEq_W - (z_pitch - z_WRrtendon);
        double z_PitchLtoe_W = (z_PitchRleq_W + z_PitchRreq_W) / 2.0;

        double x = x_rleq;
        double y = y_rleq;
        double z = z_rleq;
        double avg = z_PitchLtoe_W;
        double dev = (z_PitchRleq_W - z_PitchRreq_W) / 2.0;
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        double avg2 = avg * avg;
        double dev2 = dev * dev;
        double delta = avg2 * x2 - (x2 + z2) * (avg2 + z2 * dev2 / y2 - z2);
        if (delta < 0.0) {
            delta = 0.0;
        }
        double s_1 = (-avg * x - std::sqrt(delta)) / (x2 + z2);
        q11 = std::asin(s_1);
        double z_RrRleq_W = z_PitchRleq_W - z_PitchRreq_W;
        double z_RrRleq_Pitch = z_RrRleq_W / std::cos(q11);
        q12 = std::asin(z_RrRleq_Pitch / (y_rleq - y_rreq));
        if ((std::abs(q11 - q110) + std::abs(q12 - q120)) < 1e-6) {
            break;
        }
}

Eigen::VectorXd result(12);
result << q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12;
return result;
}

Eigen::VectorXd AnkleSolver::joint_to_motor_position_pro_(const Eigen::VectorXd& q)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double z_llbar = config[7]; 
    double z_lrbar = config[8];
    double x_lltd = config[9]; double y_lltd = config[10]; double z_lltd = config[11];
    double x_lrtd = config[12]; double y_lrtd = config[13]; double z_lrtd = config[14];
    double l_llbar = config[15];
    double l_lrbar = config[16];
    double l_lltd = config[17];
    double l_lrtd = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double z_rlbar = config[25];
    double z_rrbar = config[26];
    double x_rltd = config[27]; double y_rltd = config[28]; double z_rltd = config[29];
    double x_rrtd = config[30]; double y_rrtd = config[31]; double z_rrtd = config[32];
    double l_rlbar = config[33];
    double l_rrbar = config[34];
    double l_rltd = config[35];
    double l_rrtd = config[36];

    double z_BarKnee = config[37];
    double l_tendon = config[38];
    double l_BarTd = config[39];
    double l_KneeEq = config[40];
    double qO_knee = config[41];
    double qO_bar = config[42];

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double q4 = q(3);
    double q5 = q(4);
    double q6 = q(5);
    double q7 = q(6);
    double q8 = q(7);
    double q9 = q(8);
    double q10 = q(9);
    double q11 = q(10);
    double q12 = q(11);

    // left
    double p1 = q1;
    double p2 = q2;
    double p3 = q3;

    // left knee
    double sq4 = std::sin(q4 + qO_knee);
    double cq4 = std::cos(q4 + qO_knee);
    double x_BarEq_W = l_KneeEq * sq4;
    double z_BarEq_W = z_BarKnee + l_KneeEq * cq4;
    double l_BarEq = std::sqrt(x_BarEq_W * x_BarEq_W + z_BarEq_W * z_BarEq_W);
    double p4 = M_PI - std::acos((l_BarTd * l_BarTd + l_BarEq * l_BarEq - l_tendon * l_tendon) / (2 * l_BarTd * l_BarEq)) + std::atan2(x_BarEq_W, -z_BarEq_W) - qO_bar;

    // left foot
    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double y_LlbarEq_W = y_lleq * cq6 - z_lleq * sq6;
    double z_LlbarEq_W = -x_lleq * sq5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 + z_pitch;
    double y_LrbarEq_W = y_lreq * cq6 - z_lreq * sq6;
    double z_LrbarEq_W = -x_lreq * sq5 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 + z_pitch;
    double x_LltdEq_W = x_lleq * cq5 - x_lltd + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6;
    double x_LrtdEq_W = x_lreq * cq5 - x_lrtd + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6;
    double b_ll = std::sqrt(y_LlbarEq_W * y_LlbarEq_W + z_LlbarEq_W * z_LlbarEq_W);
    double a_ll = l_llbar;
    double c_ll = std::sqrt(l_lltd * l_lltd - x_LltdEq_W * x_LltdEq_W);
    double p6 = std::atan2(z_LlbarEq_W, y_LlbarEq_W) + std::acos((a_ll * a_ll + b_ll * b_ll - c_ll * c_ll) / (2 * a_ll * b_ll)) - std::atan2(z_lltd, y_lltd);
    double b_lr = std::sqrt(y_LrbarEq_W * y_LrbarEq_W + z_LrbarEq_W * z_LrbarEq_W);
    double a_lr = l_lrbar;
    double c_lr = std::sqrt(l_lrtd * l_lrtd - x_LrtdEq_W * x_LrtdEq_W);
    double p5 = std::atan2(z_LrbarEq_W, y_LrbarEq_W) - std::acos((a_lr * a_lr + b_lr * b_lr - c_lr * c_lr) / (2 * a_lr * b_lr)) - std::atan2(z_lrtd, y_lrtd);

    // right
    double p7 = q7;
    double p8 = q8;
    double p9 = q9;

    // right knee
    double sq10 = std::sin(q10 + qO_knee);
    double cq10 = std::cos(q10 + qO_knee);
    x_BarEq_W = l_KneeEq * sq10;
    z_BarEq_W = z_BarKnee + l_KneeEq * cq10;
    l_BarEq = std::sqrt(x_BarEq_W * x_BarEq_W + z_BarEq_W * z_BarEq_W);
    double p10 = M_PI - std::acos((l_BarTd * l_BarTd + l_BarEq * l_BarEq - l_tendon * l_tendon) / (2 * l_BarTd * l_BarEq)) + std::atan2(x_BarEq_W, -z_BarEq_W) - qO_bar;

    // right foot
    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double y_RlbarEq_W = y_rleq * cq12 - z_rleq * sq12;
    double z_RlbarEq_W = -x_rleq * sq11 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 + z_pitch;
    double y_RrbarEq_W = y_rreq * cq12 - z_rreq * sq12;
    double z_RrbarEq_W = -x_rreq * sq11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 + z_pitch;
    double x_RltdEq_W = x_rleq * cq11 - x_rltd + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12;
    double x_RrtdEq_W = x_rreq * cq11 - x_rrtd + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12;
    double b_rl = std::sqrt(y_RlbarEq_W * y_RlbarEq_W + z_RlbarEq_W * z_RlbarEq_W);
    double a_rl = l_rlbar;
    double c_rl = std::sqrt(l_rltd * l_rltd - x_RltdEq_W * x_RltdEq_W);
    double p11 = std::atan2(z_RlbarEq_W, y_RlbarEq_W) + std::acos((a_rl * a_rl + b_rl * b_rl - c_rl * c_rl) / (2 * a_rl * b_rl)) - std::atan2(z_rltd, y_rltd);
    double b_rr = std::sqrt(y_RrbarEq_W * y_RrbarEq_W + z_RrbarEq_W * z_RrbarEq_W);
    double a_rr = l_rrbar;
    double c_rr = std::sqrt(l_rrtd * l_rrtd - x_RrtdEq_W * x_RrtdEq_W);
    double p12 = std::atan2(z_RrbarEq_W, y_RrbarEq_W) - std::acos((a_rr * a_rr + b_rr * b_rr - c_rr * c_rr) / (2 * a_rr * b_rr)) - std::atan2(z_rrtd, y_rrtd);

    Eigen::VectorXd result(12);
    result << p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12;
    return result;
}

Eigen::VectorXd AnkleSolver::joint_to_motor_velocity_pro_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double z_llbar = config[7]; 
    double z_lrbar = config[8];
    double x_lltd = config[9]; double y_lltd = config[10]; double z_lltd = config[11];
    double x_lrtd = config[12]; double y_lrtd = config[13]; double z_lrtd = config[14];
    double l_llbar = config[15];
    double l_lrbar = config[16];
    double l_lltd = config[17];
    double l_lrtd = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double z_rlbar = config[25];
    double z_rrbar = config[26];
    double x_rltd = config[27]; double y_rltd = config[28]; double z_rltd = config[29];
    double x_rrtd = config[30]; double y_rrtd = config[31]; double z_rrtd = config[32];
    double l_rlbar = config[33];
    double l_rrbar = config[34];
    double l_rltd = config[35];
    double l_rrtd = config[36];

    double z_BarKnee = config[37];
    double l_tendon = config[38];
    double l_BarTd = config[39];
    double l_KneeEq = config[40];
    double qO_knee = config[41];
    double qO_bar = config[42];

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double q4 = q(3);
    double q5 = q(4);
    double q6 = q(5);
    double q7 = q(6);
    double q8 = q(7);
    double q9 = q(8);
    double q10 = q(9);
    double q11 = q(10);
    double q12 = q(11);

    double dq1 = dp(0);
    double dq2 = dp(1);
    double dq3 = dp(2);
    double dq4 = dp(3);
    double dq5 = dp(4);
    double dq6 = dp(5);
    double dq7 = dp(6);
    double dq8 = dp(7);
    double dq9 = dp(8);
    double dq10 = dp(9);
    double dq11 = dp(10);
    double dq12 = dp(11);

    double p1 = p(0);
    double p2 = p(1);
    double p3 = p(2);
    double p4 = p(3);
    double p5 = p(4);
    double p6 = p(5);
    double p7 = p(6);
    double p8 = p(7);
    double p9 = p(8);
    double p10 = p(9);
    double p11 = p(10);
    double p12 = p(11);

    // left
    double v1 = dq1;
    double v2 = dq2;
    double v3 = dq3;

    // left knee
    double sq4 = std::sin(q4 + qO_knee);
    double cq4 = std::cos(q4 + qO_knee);
    double x_BarEq_W = l_KneeEq * sq4;
    double z_BarEq_W = z_BarKnee + l_KneeEq * cq4;
    double sp4 = std::sin(-p4 - qO_bar);
    double cp4 = std::cos(-p4 - qO_bar);
    double x_BarTd_W = l_BarTd * sp4;
    double z_BarTd_W = l_BarTd * cp4;
    Eigen::Vector2d Jbar_p_WTd;
    Jbar_p_WTd << -l_BarTd * cp4, l_BarTd * sp4;
    Eigen::Vector2d Jknee_p_WEq;
    Jknee_p_WEq << l_KneeEq * cq4, -l_KneeEq * sq4;
    Eigen::Vector2d Jxx_l_Td_Vector;
    Jxx_l_Td_Vector << x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W;
    double Jknee_bar = 1.0 / (Jxx_l_Td_Vector.dot(Jbar_p_WTd)) * (Jxx_l_Td_Vector.dot(Jknee_p_WEq));
    double v4 = Jknee_bar * dq4;

    // left foot
    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                 0, -y_lleq * sq6 - z_lleq * cq6,
                 -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                 -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                 0, -y_lreq * sq6 - z_lreq * cq6,
                 -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << 0, 0,
                 -y_lltd * sp6 - z_lltd * cp6, 0,
                 y_lltd * cp6 - z_lltd * sp6, 0,
                 0, 0,
                 0, -y_lrtd * sp5 - z_lrtd * cp5,
                 0, y_lrtd * cp5 - z_lrtd * sp5;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6,
                y_lleq * cq6 - y_lltd * cp6 - z_lleq * sq6 + z_lltd * sp6,
                -x_lleq * sq5 + y_lleq * sq6 * cq5 - y_lltd * sp6 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp6 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6,
                y_lreq * cq6 - y_lrtd * cp5 - z_lreq * sq6 + z_lrtd * sp5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 - y_lrtd * sp5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp5 + z_pitch;

    Eigen::Vector2d AkDt;
    AkDt << dq5, dq6;
    Eigen::Vector2d ActDt = (Jxx_l_Td * JAct_p_WTd).colPivHouseholderQr().solve(Jxx_l_Td * JAk_p_WEq * AkDt);
    double v6 = ActDt(0);
    double v5 = ActDt(1);

    // right
    double v7 = dq7;
    double v8 = dq8;
    double v9 = dq9;

    // right knee
    double sq10 = std::sin(q10 + qO_knee);
    double cq10 = std::cos(q10 + qO_knee);
    x_BarEq_W = l_KneeEq * sq10;
    z_BarEq_W = z_BarKnee + l_KneeEq * cq10;
    double sp10 = std::sin(-p10 - qO_bar);
    double cp10 = std::cos(-p10 - qO_bar);
    x_BarTd_W = l_BarTd * sp10;
    z_BarTd_W = l_BarTd * cp10;
    Eigen::Vector2d Jbar_p_WTd_right;
    Jbar_p_WTd_right << -l_BarTd * cp10, l_BarTd * sp10;
    Eigen::Vector2d Jknee_p_WEq_right;
    Jknee_p_WEq_right << l_KneeEq * cq10, -l_KneeEq * sq10;
    Eigen::Vector2d Jxx_l_Td_right_Vector;
    Jxx_l_Td_right_Vector << x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W;
    double Jknee_bar_right = 1.0 / (Jxx_l_Td_right_Vector.dot(Jbar_p_WTd_right)) * (Jxx_l_Td_right_Vector.dot(Jknee_p_WEq_right));
    double v10 = Jknee_bar_right * dq10;

    // right foot
    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                       0, -y_rleq * sq12 - z_rleq * cq12,
                       -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                       -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                       0, -y_rreq * sq12 - z_rreq * cq12,
                       -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << 0, 0,
                        -y_rltd * sp11 - z_rltd * cp11, 0,
                        y_rltd * cp11 - z_rltd * sp11, 0,
                        0, 0,
                        0, -y_rrtd * sp12 - z_rrtd * cp12,
                        0, y_rrtd * cp12 - z_rrtd * sp12;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12,
                      y_rleq * cq12 - y_rltd * cp11 - z_rleq * sq12 + z_rltd * sp11,
                      -x_rleq * sq11 + y_rleq * sq12 * cq11 - y_rltd * sp11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp11 + z_pitch,
                      0, 0, 0,
                      0, 0, 0,
                      x_rreq * cq11 - x_rrtd + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12,
                      y_rreq * cq12 - y_rrtd * cp12 - z_rreq * sq12 + z_rrtd * sp12,
                      -x_rreq * sq11 + y_rreq * sq12 * cq11 - y_rrtd * sp12 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp12 + z_pitch;

    Eigen::Vector2d AkDt_right;
    AkDt_right << dq11, dq12;
    Eigen::Vector2d ActDt_right = (Jxx_l_Td_right * JAct_p_WTd_right).colPivHouseholderQr().solve(Jxx_l_Td_right * JAk_p_WEq_right * AkDt_right);
    double v11 = ActDt_right(0);
    double v12 = ActDt_right(1);

    Eigen::VectorXd result (12);
    result << v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
    return result;
}

Eigen::VectorXd AnkleSolver::joint_to_motor_current_pro_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double z_llbar = config[7]; 
    double z_lrbar = config[8];
    double x_lltd = config[9]; double y_lltd = config[10]; double z_lltd = config[11];
    double x_lrtd = config[12]; double y_lrtd = config[13]; double z_lrtd = config[14];
    double l_llbar = config[15];
    double l_lrbar = config[16];
    double l_lltd = config[17];
    double l_lrtd = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double z_rlbar = config[25];
    double z_rrbar = config[26];
    double x_rltd = config[27]; double y_rltd = config[28]; double z_rltd = config[29];
    double x_rrtd = config[30]; double y_rrtd = config[31]; double z_rrtd = config[32];
    double l_rlbar = config[33];
    double l_rrbar = config[34];
    double l_rltd = config[35];
    double l_rrtd = config[36];

    double z_BarKnee = config[37];
    double l_tendon = config[38];
    double l_BarTd = config[39];
    double l_KneeEq = config[40];
    double qO_knee = config[41];
    double qO_bar = config[42];

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double q4 = q(3);
    double q5 = q(4);
    double q6 = q(5);
    double q7 = q(6);
    double q8 = q(7);
    double q9 = q(8);
    double q10 = q(9);
    double q11 = q(10);
    double q12 = q(11);

    double p1 = p(0);
    double p2 = p(1);
    double p3 = p(2);
    double p4 = p(3);
    double p5 = p(4);
    double p6 = p(5);
    double p7 = p(6);
    double p8 = p(7);
    double p9 = p(8);
    double p10 = p(9);
    double p11 = p(10);
    double p12 = p(11);

    double t1 = t(0);
    double t2 = t(1);
    double t3 = t(2);
    double t4 = t(3);
    double t5 = t(4);
    double t6 = t(5);
    double t7 = t(6);
    double t8 = t(7);
    double t9 = t(8);
    double t10 = t(9);
    double t11 = t(10);
    double t12 = t(11);

    // left
    double iq1 = t1;
    double iq2 = t2;
    double iq3 = t3;

    // left knee
    double sq4 = std::sin(q4 + qO_knee);
    double cq4 = std::cos(q4 + qO_knee);
    double x_BarEq_W = l_KneeEq * sq4;
    double z_BarEq_W = z_BarKnee + l_KneeEq * cq4;
    double sp4 = std::sin(-p4 - qO_bar);
    double cp4 = std::cos(-p4 - qO_bar);
    double x_BarTd_W = l_BarTd * sp4;
    double z_BarTd_W = l_BarTd * cp4;
    Eigen::Vector2d Jbar_p_WTd;
    Jbar_p_WTd << -l_BarTd * cp4, l_BarTd * sp4;
    Eigen::Vector2d Jknee_p_WEq;
    Jknee_p_WEq << l_KneeEq * cq4, -l_KneeEq * sq4;
    Eigen::Vector2d Jxx_l_Td_Vector;
    Jxx_l_Td_Vector << x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W;
    double Jbar_knee = 1.0 / (Jxx_l_Td_Vector.dot(Jknee_p_WEq)) * (Jxx_l_Td_Vector.dot(Jbar_p_WTd));
    double iq4 = Jbar_knee * t4;

    // left foot
    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                 0, -y_lleq * sq6 - z_lleq * cq6,
                 -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                 -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                 0, -y_lreq * sq6 - z_lreq * cq6,
                 -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << 0, 0,
                 -y_lltd * sp6 - z_lltd * cp6, 0,
                 y_lltd * cp6 - z_lltd * sp6, 0,
                 0, 0,
                 0, -y_lrtd * sp5 - z_lrtd * cp5,
                 0, y_lrtd * cp5 - z_lrtd * sp5;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6,
                y_lleq * cq6 - y_lltd * cp6 - z_lleq * sq6 + z_lltd * sp6,
                -x_lleq * sq5 + y_lleq * sq6 * cq5 - y_lltd * sp6 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp6 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6,
                y_lreq * cq6 - y_lrtd * cp5 - z_lreq * sq6 + z_lrtd * sp5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 - y_lrtd * sp5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp5 + z_pitch;
    Eigen::MatrixXd JAct_Ak = (Jxx_l_Td * JAk_p_WEq).inverse() * (Jxx_l_Td * JAct_p_WTd);
    Eigen::Vector2d tau_Ak;
    tau_Ak << t5, t6;
    Eigen::Vector2d tau_Act = JAct_Ak.transpose() * tau_Ak;
    double iq6 = tau_Act(0);
    double iq5 = tau_Act(1);

    // right
    double iq7 = t7;
    double iq8 = t8;
    double iq9 = t9;

    // right knee
    double sq10 = std::sin(q10 + qO_knee);
    double cq10 = std::cos(q10 + qO_knee);
    x_BarEq_W = l_KneeEq * sq10;
    z_BarEq_W = z_BarKnee + l_KneeEq * cq10;
    double sp10 = std::sin(-p10 - qO_bar);
    double cp10 = std::cos(-p10 - qO_bar);
    x_BarTd_W = l_BarTd * sp10;
    z_BarTd_W = l_BarTd * cp10;
    Eigen::Vector2d Jbar_p_WTd_right;
    Jbar_p_WTd_right << -l_BarTd * cp10, l_BarTd * sp10;
    Eigen::Vector2d Jknee_p_WEq_right;
    Jknee_p_WEq_right << l_KneeEq * cq10, -l_KneeEq * sq10;
    Eigen::Vector2d Jxx_l_Td_right_Vector;
    Jxx_l_Td_right_Vector << x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W;
    double Jbar_knee_right = 1.0 / (Jxx_l_Td_right_Vector.dot(Jknee_p_WEq_right)) * (Jxx_l_Td_right_Vector.dot(Jbar_p_WTd_right));
    double iq10 = Jbar_knee_right * t10;

    // right foot
    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                       0, -y_rleq * sq12 - z_rleq * cq12,
                       -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                       -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                       0, -y_rreq * sq12 - z_rreq * cq12,
                       -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << 0, 0,
                        -y_rltd * sp11 - z_rltd * cp11, 0,
                        y_rltd * cp11 - z_rltd * sp11, 0,
                        0, 0,
                        0, -y_rrtd * sp12 - z_rrtd * cp12,
                        0, y_rrtd * cp12 - z_rrtd * sp12;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12,
                      y_rleq * cq12 - y_rltd * cp11 - z_rleq * sq12 + z_rltd * sp11,
                      -x_rleq * sq11 + y_rleq * sq12 * cq11 - y_rltd * sp11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp11 + z_pitch,
                      0, 0, 0,
                      0, 0, 0,
                      x_rreq * cq11 - x_rrtd + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12,
                      y_rreq * cq12 - y_rrtd * cp12 - z_rreq * sq12 + z_rrtd * sp12,
                      -x_rreq * sq11 + y_rreq * sq12 * cq11 - y_rrtd * sp12 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp12 + z_pitch;

    Eigen::MatrixXd JAct_Ak_right = (Jxx_l_Td_right * JAk_p_WEq_right).inverse() * (Jxx_l_Td_right * JAct_p_WTd_right);
    Eigen::Vector2d tau_Ak_right;
    tau_Ak_right << t11, t12;
    Eigen::Vector2d tau_Act_right = JAct_Ak_right.transpose() * tau_Ak_right;
    double iq11 = tau_Act_right(0);
    double iq12 = tau_Act_right(1);

    Eigen::VectorXd result(12);
    result << iq1, iq2, iq3, iq4, iq5, iq6, iq7, iq8, iq9, iq10, iq11, iq12;
    return result;
}

Eigen::VectorXd AnkleSolver::motor_to_joint_position_pro_(const Eigen::VectorXd& p)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double z_llbar = config[7]; 
    double z_lrbar = config[8];
    double x_lltd = config[9]; double y_lltd = config[10]; double z_lltd = config[11];
    double x_lrtd = config[12]; double y_lrtd = config[13]; double z_lrtd = config[14];
    double l_llbar = config[15];
    double l_lrbar = config[16];
    double l_lltd = config[17];
    double l_lrtd = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double z_rlbar = config[25];
    double z_rrbar = config[26];
    double x_rltd = config[27]; double y_rltd = config[28]; double z_rltd = config[29];
    double x_rrtd = config[30]; double y_rrtd = config[31]; double z_rrtd = config[32];
    double l_rlbar = config[33];
    double l_rrbar = config[34];
    double l_rltd = config[35];
    double l_rrtd = config[36];

    double z_BarKnee = config[37];
    double l_tendon = config[38];
    double l_BarTd = config[39];
    double l_KneeEq = config[40];
    double qO_knee = config[41];
    double qO_bar = config[42];

    double p1 = p(0);
    double p2 = p(1);
    double p3 = p(2);
    double p4 = p(3);
    double p5 = p(4);
    double p6 = p(5);
    double p7 = p(6);
    double p8 = p(7);
    double p9 = p(8);
    double p10 = p(9);
    double p11 = p(10);
    double p12 = p(11);

    // left
    double q1 = p1;
    double q2 = p2;
    double q3 = p3;

    // left knee
    double sp4 = std::sin(-p4 - qO_bar);
    double cp4 = std::cos(-p4 - qO_bar);
    double x_KneeTd_W = l_BarTd * sp4;
    double z_KneeTd_W = l_BarTd * cp4 - z_BarKnee;
    double l_KneeTd = std::sqrt(x_KneeTd_W * x_KneeTd_W + z_KneeTd_W * z_KneeTd_W);
    double q4 = std::acos((l_KneeEq * l_KneeEq + l_KneeTd * l_KneeTd - l_tendon * l_tendon) / (2 * l_KneeEq * l_KneeTd)) - std::atan2(-x_KneeTd_W, z_KneeTd_W) - qO_knee;

    // left foot
    double q5 = 0.0, q6 = 0.0;
    for (int i = 0; i < N_ITER; ++i) {
        double q50 = q5, q60 = q6;
        double cq50 = std::cos(q50);
        double sq50 = std::sin(q50);
        double cq60 = std::cos(q60);
        double sq60 = std::sin(q60);
        double cp6 = std::cos(p6);
        double sp6 = std::sin(p6);
        double cp5 = std::cos(p5);
        double sp5 = std::sin(p5);

        double x_LltendonEq_W = x_lleq * cq50 - x_lltd + y_lleq * sq50 * sq60 + z_lleq * sq50 * cq60;
        double y_LltendonEq_W = y_lleq * cq60 - y_lltd * cp6 - z_lleq * sq60 + z_lltd * sp6;
        double z_LltendonEq_W = -std::sqrt(l_lltd * l_lltd - x_LltendonEq_W * x_LltendonEq_W - y_LltendonEq_W * y_LltendonEq_W);
        double z_WLltendon = y_lltd * sp6 + z_llbar + z_lltd * cp6;
        double z_PitchLleq_W = z_LltendonEq_W - (z_pitch - z_WLltendon);
        double x_LrtendonEq_W = x_lreq * cq50 - x_lrtd + y_lreq * sq50 * sq60 + z_lreq * sq50 * cq60;
        double y_LrtendonEq_W = y_lreq * cq60 - y_lrtd * cp5 - z_lreq * sq60 + z_lrtd * sp5;
        double z_LrtendonEq_W = -std::sqrt(l_lrtd * l_lrtd - x_LrtendonEq_W * x_LrtendonEq_W - y_LrtendonEq_W * y_LrtendonEq_W);
        double z_WLrtendon = y_lrtd * sp5 + z_lrbar + z_lrtd * cp5;
        double z_PitchLreq_W = z_LrtendonEq_W - (z_pitch - z_WLrtendon);
        double z_PitchLtoe_W = (z_PitchLleq_W + z_PitchLreq_W) / 2.0;

        double x = x_lleq;
        double y = y_lleq;
        double z = z_lleq;
        double avg = z_PitchLtoe_W;
        double dev = (z_PitchLleq_W - z_PitchLreq_W) / 2.0;
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        double avg2 = avg * avg;
        double dev2 = dev * dev;
        double delta = avg2 * x2 - (x2 + z2) * (avg2 + z2 * dev2 / y2 - z2);
        double sq5 = (-avg * x - std::sqrt(delta)) / (x2 + z2);
        q5 = std::asin(sq5);
        double z_LrLleq_W = z_PitchLleq_W - z_PitchLreq_W;
        double z_LrLleq_Pitch = z_LrLleq_W / std::cos(q5);
        q6 = std::asin(z_LrLleq_Pitch / (y_lleq - y_lreq));

        if ((std::abs(q5 - q50) + std::abs(q6 - q60)) < 1e-6) {
            break;
        }
    }

    // right
    double q7 = p7;
    double q8 = p8;
    double q9 = p9;

    // right knee
    double sp10 = std::sin(-p10 - qO_bar);
    double cp10 = std::cos(-p10 - qO_bar);
    x_KneeTd_W = l_BarTd * sp10;
    z_KneeTd_W = l_BarTd * cp10 - z_BarKnee;
    l_KneeTd = std::sqrt(x_KneeTd_W * x_KneeTd_W + z_KneeTd_W * z_KneeTd_W);
    double q10 = std::acos((l_KneeEq * l_KneeEq + l_KneeTd * l_KneeTd - l_tendon * l_tendon) / (2 * l_KneeEq * l_KneeTd)) - std::atan2(-x_KneeTd_W, z_KneeTd_W) - qO_knee;

    // right foot
    double q11 = 0.0, q12 = 0.0;
    for (int i = 0; i < N_ITER; ++i) {
        double q110 = q11, q120 = q12;
        double cq110 = std::cos(q110);
        double sq110 = std::sin(q110);
        double cq120 = std::cos(q120);
        double sq120 = std::sin(q120);
        double cp11 = std::cos(p11);
        double sp11 = std::sin(p11);
        double cp12 = std::cos(p12);
        double sp12 = std::sin(p12);

        double x_RltendonEq_W = x_rleq * cq110 - x_rltd + y_rleq * sq110 * sq120 + z_rleq * sq110 * cq120;
        double y_RltendonEq_W = y_rleq * cq120 - y_rltd * cp11 - z_rleq * sq120 + z_rltd * sp11;
        double z_RltendonEq_W = -std::sqrt(l_rltd * l_rltd - x_RltendonEq_W * x_RltendonEq_W - y_RltendonEq_W * y_RltendonEq_W);
        double z_WRltendon = y_rltd * sp11 + z_rlbar + z_rltd * cp11;
        double z_PitchRleq_W = z_RltendonEq_W - (z_pitch - z_WRltendon);
        double x_RrtendonEq_W = x_rreq * cq110 - x_rrtd + y_rreq * sq110 * sq120 + z_rreq * sq110 * cq120;
        double y_RrtendonEq_W = y_rreq * cq120 - y_rrtd * cp12 - z_rreq * sq120 + z_rrtd * sp12;
        double z_RrtendonEq_W = -std::sqrt(l_rrtd * l_rrtd - x_RrtendonEq_W * x_RrtendonEq_W - y_RrtendonEq_W * y_RrtendonEq_W);
        double z_WRrtendon = y_rrtd * sp12 + z_rrbar + z_rrtd * cp12;
        double z_PitchRreq_W = z_RrtendonEq_W - (z_pitch - z_WRrtendon);
        double z_PitchLtoe_W = (z_PitchRleq_W + z_PitchRreq_W) / 2.0;

        double x = x_rleq;
        double y = y_rleq;
        double z = z_rleq;
        double avg = z_PitchLtoe_W;
        double dev = (z_PitchRleq_W - z_PitchRreq_W) / 2.0;
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        double avg2 = avg * avg;
        double dev2 = dev * dev;
        double delta = avg2 * x2 - (x2 + z2) * (avg2 + z2 * dev2 / y2 - z2);
        double sq11 = (-avg * x - std::sqrt(delta)) / (x2 + z2);
        q11 = std::asin(sq11);
        double z_RrRleq_W = z_PitchRleq_W - z_PitchRreq_W;
        double z_RrRleq_Pitch = z_RrRleq_W / std::cos(q11);
        q12 = std::asin(z_RrRleq_Pitch / (y_rleq - y_rreq));

        if ((std::abs(q11 - q110) + std::abs(q12 - q120)) < 1e-6) {
            break;
        }
    }

    Eigen::VectorXd result(12);
    result << q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12;
    return result;
}

Eigen::VectorXd AnkleSolver::motor_to_joint_velocity_pro_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double z_llbar = config[7]; 
    double z_lrbar = config[8];
    double x_lltd = config[9]; double y_lltd = config[10]; double z_lltd = config[11];
    double x_lrtd = config[12]; double y_lrtd = config[13]; double z_lrtd = config[14];
    double l_llbar = config[15];
    double l_lrbar = config[16];
    double l_lltd = config[17];
    double l_lrtd = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double z_rlbar = config[25];
    double z_rrbar = config[26];
    double x_rltd = config[27]; double y_rltd = config[28]; double z_rltd = config[29];
    double x_rrtd = config[30]; double y_rrtd = config[31]; double z_rrtd = config[32];
    double l_rlbar = config[33];
    double l_rrbar = config[34];
    double l_rltd = config[35];
    double l_rrtd = config[36];

    double z_BarKnee = config[37];
    double l_tendon = config[38];
    double l_BarTd = config[39];
    double l_KneeEq = config[40];
    double qO_knee = config[41];
    double qO_bar = config[42];

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double q4 = q(3);
    double q5 = q(4);
    double q6 = q(5);
    double q7 = q(6);
    double q8 = q(7);
    double q9 = q(8);
    double q10 = q(9);
    double q11 = q(10);
    double q12 = q(11);

    double p1 = p(0);
    double p2 = p(1);
    double p3 = p(2);
    double p4 = p(3);
    double p5 = p(4);
    double p6 = p(5);
    double p7 = p(6);
    double p8 = p(7);
    double p9 = p(8);
    double p10 = p(9);
    double p11 = p(10);
    double p12 = p(11);

    double v1 = v(0);
    double v2 = v(1);
    double v3 = v(2);
    double v4 = v(3);
    double v5 = v(4);
    double v6 = v(5);
    double v7 = v(6);
    double v8 = v(7);
    double v9 = v(8);
    double v10 = v(9);
    double v11 = v(10);
    double v12 = v(11);

    // left
    double dq1 = v1;
    double dq2 = v2;
    double dq3 = v3;

    // left knee
    double sq4 = std::sin(q4 + qO_knee);
    double cq4 = std::cos(q4 + qO_knee);
    double x_BarEq_W = l_KneeEq * sq4;
    double z_BarEq_W = z_BarKnee + l_KneeEq * cq4;
    double sp4 = std::sin(-p4 - qO_bar);
    double cp4 = std::cos(-p4 - qO_bar);
    double x_BarTd_W = l_BarTd * sp4;
    double z_BarTd_W = l_BarTd * cp4;
    Eigen::Vector2d Jbar_p_WTd;
    Jbar_p_WTd << -l_BarTd * cp4, l_BarTd * sp4;
    Eigen::Vector2d Jknee_p_WEq;
    Jknee_p_WEq << l_KneeEq * cq4, -l_KneeEq * sq4;
    Eigen::Vector2d Jxx_l_Td_Vector;
    Jxx_l_Td_Vector << x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W;
    double Jbar_knee = 1.0 / (Jxx_l_Td_Vector.dot(Jknee_p_WEq)) * (Jxx_l_Td_Vector.dot(Jbar_p_WTd));
    double dq4 = Jbar_knee * v4;

    // left foot
    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                 0, -y_lleq * sq6 - z_lleq * cq6,
                 -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                 -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                 0, -y_lreq * sq6 - z_lreq * cq6,
                 -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << 0, 0,
                 -y_lltd * sp6 - z_lltd * cp6, 0,
                 y_lltd * cp6 - z_lltd * sp6, 0,
                 0, 0,
                 0, -y_lrtd * sp5 - z_lrtd * cp5,
                 0, y_lrtd * cp5 - z_lrtd * sp5;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6,
                y_lleq * cq6 - y_lltd * cp6 - z_lleq * sq6 + z_lltd * sp6,
                -x_lleq * sq5 + y_lleq * sq6 * cq5 - y_lltd * sp6 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp6 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6,
                y_lreq * cq6 - y_lrtd * cp5 - z_lreq * sq6 + z_lrtd * sp5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 - y_lrtd * sp5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp5 + z_pitch;

    Eigen::Vector2d ActDt;
    ActDt << v6, v5;
    Eigen::Vector2d AkDt = (Jxx_l_Td * JAk_p_WEq).colPivHouseholderQr().solve(Jxx_l_Td * JAct_p_WTd * ActDt);
    double dq5 = AkDt(0);
    double dq6 = AkDt(1);

    // right
    double dq7 = v7;
    double dq8 = v8;
    double dq9 = v9;

    // right knee
    double sq10 = std::sin(q10 + qO_knee);
    double cq10 = std::cos(q10 + qO_knee);
    x_BarEq_W = l_KneeEq * sq10;
    z_BarEq_W = z_BarKnee + l_KneeEq * cq10;
    double sp10 = std::sin(-p10 - qO_bar);
    double cp10 = std::cos(-p10 - qO_bar);
    x_BarTd_W = l_BarTd * sp10;
    z_BarTd_W = l_BarTd * cp10;
    Eigen::Vector2d Jbar_p_WTd_right;
    Jbar_p_WTd_right << -l_BarTd * cp10, l_BarTd * sp10;
    Eigen::Vector2d Jknee_p_WEq_right;
    Jknee_p_WEq_right << l_KneeEq * cq10, -l_KneeEq * sq10;
    Eigen::Vector2d Jxx_l_Td_right_Vector;
    Jxx_l_Td_right_Vector << x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W;
    double Jbar_knee_right = 1.0 / (Jxx_l_Td_right_Vector.dot(Jknee_p_WEq_right)) * (Jxx_l_Td_right_Vector.dot(Jbar_p_WTd_right));
    double dq10 = Jbar_knee_right * v10;

    // right foot
    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                       0, -y_rleq * sq12 - z_rleq * cq12,
                       -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                       -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                       0, -y_rreq * sq12 - z_rreq * cq12,
                       -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << 0, 0,
                        -y_rltd * sp11 - z_rltd * cp11, 0,
                        y_rltd * cp11 - z_rltd * sp11, 0,
                        0, 0,
                        0, -y_rrtd * sp12 - z_rrtd * cp12,
                        0, y_rrtd * cp12 - z_rrtd * sp12;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12,
                      y_rleq * cq12 - y_rltd * cp11 - z_rleq * sq12 + z_rltd * sp11,
                      -x_rleq * sq11 + y_rleq * sq12 * cq11 - y_rltd * sp11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp11 + z_pitch,
                      0, 0, 0,
                      0, 0, 0,
                      x_rreq * cq11 - x_rrtd + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12,
                      y_rreq * cq12 - y_rrtd * cp12 - z_rreq * sq12 + z_rrtd * sp12,
                      -x_rreq * sq11 + y_rreq * sq12 * cq11 - y_rrtd * sp12 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp12 + z_pitch;

    Eigen::Vector2d ActDt_right;
    ActDt_right << v11, v12;
    Eigen::Vector2d AkDt_right = (Jxx_l_Td_right * JAk_p_WEq_right).colPivHouseholderQr().solve(Jxx_l_Td_right * JAct_p_WTd_right * ActDt_right);
    double dq11 = AkDt_right(0);
    double dq12 = AkDt_right(1);

    Eigen::VectorXd result(12);
    result << dq1, dq2, dq3, dq4, dq5, dq6, dq7, dq8, dq9, dq10, dq11, dq12;
    return result;
}

Eigen::VectorXd AnkleSolver::motor_to_joint_torque_pro_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double z_llbar = config[7]; 
    double z_lrbar = config[8];
    double x_lltd = config[9]; double y_lltd = config[10]; double z_lltd = config[11];
    double x_lrtd = config[12]; double y_lrtd = config[13]; double z_lrtd = config[14];
    double l_llbar = config[15];
    double l_lrbar = config[16];
    double l_lltd = config[17];
    double l_lrtd = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double z_rlbar = config[25];
    double z_rrbar = config[26];
    double x_rltd = config[27]; double y_rltd = config[28]; double z_rltd = config[29];
    double x_rrtd = config[30]; double y_rrtd = config[31]; double z_rrtd = config[32];
    double l_rlbar = config[33];
    double l_rrbar = config[34];
    double l_rltd = config[35];
    double l_rrtd = config[36];

    double z_BarKnee = config[37];
    double l_tendon = config[38];
    double l_BarTd = config[39];
    double l_KneeEq = config[40];
    double qO_knee = config[41];
    double qO_bar = config[42];
    
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double q4 = q(3);
    double q5 = q(4);
    double q6 = q(5);
    double q7 = q(6);
    double q8 = q(7);
    double q9 = q(8);
    double q10 = q(9);
    double q11 = q(10);
    double q12 = q(11);

    double p1 = p(0);
    double p2 = p(1);
    double p3 = p(2);
    double p4 = p(3);
    double p5 = p(4);
    double p6 = p(5);
    double p7 = p(6);
    double p8 = p(7);
    double p9 = p(8);
    double p10 = p(9);
    double p11 = p(10);
    double p12 = p(11);

    double i1 = c(0);
    double i2 = c(1);
    double i3 = c(2);
    double i4 = c(3);
    double i5 = c(4);
    double i6 = c(5);
    double i7 = c(6);
    double i8 = c(7);
    double i9 = c(8);
    double i10 = c(9);
    double i11 = c(10);
    double i12 = c(11);

    // left
    double t1 = i1;
    double t2 = i2;
    double t3 = i3;

    // left knee
    double sq4 = std::sin(q4 + qO_knee);
    double cq4 = std::cos(q4 + qO_knee);
    double x_BarEq_W = l_KneeEq * sq4;
    double z_BarEq_W = z_BarKnee + l_KneeEq * cq4;
    double sp4 = std::sin(-p4 - qO_bar);
    double cp4 = std::cos(-p4 - qO_bar);
    double x_BarTd_W = l_BarTd * sp4;
    double z_BarTd_W = l_BarTd * cp4;
    Eigen::Vector2d Jbar_p_WTd;
    Jbar_p_WTd << -l_BarTd * cp4, l_BarTd * sp4;
    Eigen::Vector2d Jknee_p_WEq;
    Jknee_p_WEq << l_KneeEq * cq4, -l_KneeEq * sq4;
    Eigen::Vector2d Jxx_l_Td_Vector;
    Jxx_l_Td_Vector << x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W;
    double Jknee_bar = 1.0 / (Jxx_l_Td_Vector.dot(Jbar_p_WTd)) * (Jxx_l_Td_Vector.dot(Jknee_p_WEq));
    double t4 = Jknee_bar * i4;

    // left foot
    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                 0, -y_lleq * sq6 - z_lleq * cq6,
                 -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                 -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                 0, -y_lreq * sq6 - z_lreq * cq6,
                 -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << 0, 0,
                 -y_lltd * sp6 - z_lltd * cp6, 0,
                 y_lltd * cp6 - z_lltd * sp6, 0,
                 0, 0,
                 0, -y_lrtd * sp5 - z_lrtd * cp5,
                 0, y_lrtd * cp5 - z_lrtd * sp5;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6,
                y_lleq * cq6 - y_lltd * cp6 - z_lleq * sq6 + z_lltd * sp6,
                -x_lleq * sq5 + y_lleq * sq6 * cq5 - y_lltd * sp6 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp6 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6,
                y_lreq * cq6 - y_lrtd * cp5 - z_lreq * sq6 + z_lrtd * sp5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 - y_lrtd * sp5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp5 + z_pitch;

    Eigen::MatrixXd JAk_Act = (Jxx_l_Td * JAct_p_WTd).inverse() * (Jxx_l_Td * JAk_p_WEq);
    Eigen::Vector2d tau_Act;
    tau_Act << i6, i5;
    Eigen::Vector2d tau_Ak = JAk_Act.transpose() * tau_Act;
    double t5 = tau_Ak(0);
    double t6 = tau_Ak(1);

    // right
    double t7 = i7;
    double t8 = i8;
    double t9 = i9;

    // right knee
    double sq10 = std::sin(q10 + qO_knee);
    double cq10 = std::cos(q10 + qO_knee);
    x_BarEq_W = l_KneeEq * sq10;
    z_BarEq_W = z_BarKnee + l_KneeEq * cq10;
    double sp10 = std::sin(-p10 - qO_bar);
    double cp10 = std::cos(-p10 - qO_bar);
    x_BarTd_W = l_BarTd * sp10;
    z_BarTd_W = l_BarTd * cp10;
    Eigen::Vector2d Jbar_p_WTd_right;
    Jbar_p_WTd_right << -l_BarTd * cp10, l_BarTd * sp10;
    Eigen::Vector2d Jknee_p_WEq_right;
    Jknee_p_WEq_right << l_KneeEq * cq10, -l_KneeEq * sq10;
    Eigen::Vector2d Jxx_l_Td_right_Vector;
    Jxx_l_Td_right_Vector << x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W;
    double Jknee_bar_right = 1.0 / (Jxx_l_Td_right_Vector.dot(Jbar_p_WTd_right)) * (Jxx_l_Td_right_Vector.dot(Jknee_p_WEq_right));
    double t10 = Jknee_bar_right * i10;

    // right foot
    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                       0, -y_rleq * sq12 - z_rleq * cq12,
                       -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                       -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                       0, -y_rreq * sq12 - z_rreq * cq12,
                       -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << 0, 0,
                        -y_rltd * sp11 - z_rltd * cp11, 0,
                        y_rltd * cp11 - z_rltd * sp11, 0,
                        0, 0,
                        0, -y_rrtd * sp12 - z_rrtd * cp12,
                        0, y_rrtd * cp12 - z_rrtd * sp12;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12,
                      y_rleq * cq12 - y_rltd * cp11 - z_rleq * sq12 + z_rltd * sp11,
                      -x_rleq * sq11 + y_rleq * sq12 * cq11 - y_rltd * sp11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp11 + z_pitch,
                      0, 0, 0,
                      0, 0, 0,
                      x_rreq * cq11 - x_rrtd + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12,
                      y_rreq * cq12 - y_rrtd * cp12 - z_rreq * sq12 + z_rrtd * sp12,
                      -x_rreq * sq11 + y_rreq * sq12 * cq11 - y_rrtd * sp12 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp12 + z_pitch;

    Eigen::MatrixXd JAk_Act_right = (Jxx_l_Td_right * JAct_p_WTd_right).inverse() * (Jxx_l_Td_right * JAk_p_WEq_right);
    Eigen::Vector2d tau_Act_right;
    tau_Act_right << i11, i12;
    Eigen::Vector2d tau_Ak_right = JAk_Act_right.transpose() * tau_Act_right;
    double t11 = tau_Ak_right(0);
    double t12 = tau_Ak_right(1);

    Eigen::VectorXd result(12);
    result << t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
    return result;
}

/**
 * @brief foot_t => motor_c
 *
 * @param q leg joint position
 * @param p motor joint position
 * @param t leg joint torque
 */
Eigen::VectorXd AnkleSolver::joint_to_motor_current_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t) 
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[5], p6 = p[4];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[11], p12 = p[10];
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];
    double t1 = t[0], t2 = t[1], t3 = t[2], t4 = t[3], t5 = t[4], t6 = t[5];
    double t7 = t[6], t8 = t[7], t9 = t[8], t10 = t[9], t11 = t[10], t12 = t[11];

    // left
    double iq1 = t1;
    double iq2 = t2;
    double iq3 = t3;
    double iq4 = t4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                0., -y_lleq * sq6 - z_lleq * cq6,
                -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                0., -y_lreq * sq6 - z_lreq * cq6,
                -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << -x_lltd * sp5 + z_lltd * cp5, 0,
                0, 0,
                -x_lltd * cp5 - z_lltd * sp5, 0,
                0, -x_lrtd * sp6 + z_lrtd * cp6,
                0, 0,
                0, -x_lrtd * cp6 - z_lrtd * sp6;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5,
                -y_llbar + y_lleq * cq6 - z_lleq * sq6,
                -x_lleq * sq5 + x_lltd * sp5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp5 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6,
                -y_lrbar + y_lreq * cq6 - z_lreq * sq6,
                -x_lreq * sq5 + x_lrtd * sp6 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp6 + z_pitch;

    Eigen::MatrixXd JAct_Ak = (Jxx_l_Td * JAk_p_WEq).inverse() * (Jxx_l_Td * JAct_p_WTd);

    Eigen::Vector2d tau_Ak;
    tau_Ak << t5, t6;

    Eigen::Vector2d tau_Act = JAct_Ak.transpose() * tau_Ak;
    double iq5 = tau_Act[0];
    double iq6 = tau_Act[1];

    // right
    double iq7 = t7;
    double iq8 = t8;
    double iq9 = t9;
    double iq10 = t10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                    0, -y_rleq * sq12 - z_rleq * cq12,
                    -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                    -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                    0, -y_rreq * sq12 - z_rreq * cq12,
                    -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << -x_rltd * sp12 + z_rltd * cp12, 0,
                        0, 0,
                        -x_rltd * cp12 - z_rltd * sp12, 0,
                        0, -x_rrtd * sp11 + z_rrtd * cp11,
                        0, 0,
                        0, -x_rrtd * cp11 - z_rrtd * sp11;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12,
                    -y_rlbar + y_rleq * cq12 - z_rleq * sq12,
                    -x_rleq * sq11 + x_rltd * sp12 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp12 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11,
                    -y_rrbar + y_rreq * cq12 - z_rreq * sq12,
                    -x_rreq * sq11 + x_rrtd * sp11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp11 + z_pitch;

    Eigen::MatrixXd JAct_Ak_right = (Jxx_l_Td_right * JAk_p_WEq_right).inverse() * (Jxx_l_Td_right * JAct_p_WTd_right);

    Eigen::Vector2d tau_Ak_right;
    tau_Ak_right << t11, t12;

    Eigen::Vector2d tau_Act_right = JAct_Ak_right.transpose() * tau_Ak_right;
    double iq12 = tau_Act_right[0];
    double iq11 = tau_Act_right[1];

    Eigen::VectorXd result(12);
    result << iq1, iq2, iq3, iq4, iq6, iq5, iq7, iq8, iq9, iq10, iq12, iq11;
    return result;
}
/**
 * @brief foot_v => motor_v
 *
 * @param q leg joint position
 * @param p motor joint position
 * @param v leg joint velocity
 */
Eigen::VectorXd AnkleSolver::joint_to_motor_velocity_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp) 
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[5], p6 = p[4];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[11], p12 = p[10];
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];
    double dq1 = dp[0], dq2 = dp[1], dq3 = dp[2], dq4 = dp[3], dq5 = dp[4], dq6 = dp[5];
    double dq7 = dp[6], dq8 = dp[7], dq9 = dp[8], dq10 = dp[9], dq11 = dp[10], dq12 = dp[11];

    // left
    double v1 = dq1;
    double v2 = dq2;
    double v3 = dq3;
    double v4 = dq4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                0., -y_lleq * sq6 - z_lleq * cq6,
                -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                0., -y_lreq * sq6 - z_lreq * cq6,
                -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << -x_lltd * sp5 + z_lltd * cp5, 0,
                0, 0,
                -x_lltd * cp5 - z_lltd * sp5, 0,
                0, -x_lrtd * sp6 + z_lrtd * cp6,
                0, 0,
                0, -x_lrtd * cp6 - z_lrtd * sp6;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5,
                -y_llbar + y_lleq * cq6 - z_lleq * sq6,
                -x_lleq * sq5 + x_lltd * sp5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp5 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6,
                -y_lrbar + y_lreq * cq6 - z_lreq * sq6,
                -x_lreq * sq5 + x_lrtd * sp6 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp6 + z_pitch;

    Eigen::Vector2d AkDt;
    AkDt << dq5, dq6;

    Eigen::Vector2d ActDt = (Jxx_l_Td * JAct_p_WTd).colPivHouseholderQr().solve(Jxx_l_Td * JAk_p_WEq * AkDt);
    double v5 = ActDt[0];
    double v6 = ActDt[1];

    // right
    double v7 = dq7;
    double v8 = dq8;
    double v9 = dq9;
    double v10 = dq10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                    0, -y_rleq * sq12 - z_rleq * cq12,
                    -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                    -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                    0, -y_rreq * sq12 - z_rreq * cq12,
                    -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << -x_rltd * sp12 + z_rltd * cp12, 0,
                        0, 0,
                        -x_rltd * cp12 - z_rltd * sp12, 0,
                        0, -x_rrtd * sp11 + z_rrtd * cp11,
                        0, 0,
                        0, -x_rrtd * cp11 - z_rrtd * sp11;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12,
                    -y_rlbar + y_rleq * cq12 - z_rleq * sq12,
                    -x_rleq * sq11 + x_rltd * sp12 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp12 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11,
                    -y_rrbar + y_rreq * cq12 - z_rreq * sq12,
                    -x_rreq * sq11 + x_rrtd * sp11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp11 + z_pitch;

    Eigen::Vector2d AkDt_right;
    AkDt_right << dq11, dq12;

    Eigen::Vector2d ActDt_right = (Jxx_l_Td_right * JAct_p_WTd_right).colPivHouseholderQr().solve(Jxx_l_Td_right * JAk_p_WEq_right * AkDt_right);
    double v12 = ActDt_right[0];
    double v11 = ActDt_right[1];

    Eigen::VectorXd result(12);
    result << v1, v2, v3, v4, v6, v5, v7, v8, v9, v10, v12, v11;
    return result;
}
/**
 * @brief foot_q => motor_p
 *
 * @param q leg joint position
 */
Eigen::VectorXd AnkleSolver::joint_to_motor_position_s2_(const Eigen::VectorXd& q)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];

    // left
    double p1 = q1;
    double p2 = q2;
    double p3 = q3;
    double p4 = q4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);

    double x_LlbarEq_W = x_lleq * cq5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6;
    double z_LlbarEq_W = -x_lleq * sq5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 + z_pitch;
    double x_LrbarEq_W = x_lreq * cq5 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6;
    double z_LrbarEq_W = -x_lreq * sq5 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 + z_pitch;
    double y_LltdEq_W = -y_llbar + y_lleq * cq6 - z_lleq * sq6;
    double y_LrtdEq_W = -y_lrbar + y_lreq * cq6 - z_lreq * sq6;
    double b_ll = std::sqrt(x_LlbarEq_W * x_LlbarEq_W + z_LlbarEq_W * z_LlbarEq_W);
    double a_ll = l_llbar;
    double c_ll = std::sqrt(l_lltd * l_lltd - y_LltdEq_W * y_LltdEq_W);
    double p5 = std::atan2(z_LlbarEq_W, -x_LlbarEq_W) + std::acos((a_ll * a_ll + b_ll * b_ll - c_ll * c_ll) / (2 * a_ll * b_ll)) - std::atan2(z_lltd, -x_lltd);
    double b_lr = std::sqrt(x_LrbarEq_W * x_LrbarEq_W + z_LrbarEq_W * z_LrbarEq_W);
    double a_lr = l_lrbar;
    double c_lr = std::sqrt(l_lrtd * l_lrtd - y_LrtdEq_W * y_LrtdEq_W);
    double p6 = std::atan2(z_LrbarEq_W, -x_LrbarEq_W) + std::acos((a_lr * a_lr + b_lr * b_lr - c_lr * c_lr) / (2 * a_lr * b_lr)) - std::atan2(z_lrtd, -x_lrtd);

    // right
    double p7 = q7;
    double p8 = q8;
    double p9 = q9;
    double p10 = q10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);

    x_LlbarEq_W = x_rleq * cq11 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12;
    z_LlbarEq_W = -x_rleq * sq11 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 + z_pitch;
    x_LrbarEq_W = x_rreq * cq11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12;
    z_LrbarEq_W = -x_rreq * sq11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 + z_pitch;
    y_LltdEq_W = -y_rlbar + y_rleq * cq12 - z_rleq * sq12;
    y_LrtdEq_W = -y_rrbar + y_rreq * cq12 - z_rreq * sq12;
    double b_rl = std::sqrt(x_LlbarEq_W * x_LlbarEq_W + z_LlbarEq_W * z_LlbarEq_W);
    double a_rl = l_rlbar;
    double c_rl = std::sqrt(l_rltd * l_rltd - y_LltdEq_W * y_LltdEq_W);
    double p12 = std::atan2(z_LlbarEq_W, -x_LlbarEq_W) + std::acos((a_rl * a_rl + b_rl * b_rl - c_rl * c_rl) / (2 * a_rl * b_rl)) - std::atan2(z_rltd, -x_rltd);
    double b_rr = std::sqrt(x_LrbarEq_W * x_LrbarEq_W + z_LrbarEq_W * z_LrbarEq_W);
    double a_rr = l_rrbar;
    double c_rr = std::sqrt(l_rrtd * l_rrtd - y_LrtdEq_W * y_LrtdEq_W);
    double p11 = std::atan2(z_LrbarEq_W, -x_LrbarEq_W) + std::acos((a_rr * a_rr + b_rr * b_rr - c_rr * c_rr) / (2 * a_rr * b_rr)) - std::atan2(z_rrtd, -x_rrtd);

    Eigen::VectorXd result(12);
    result << p1, p2, p3, p4, p6, p5, p7, p8, p9, p10, p12, p11;
    return result;
}

/**
 * @brief motor_c => foot_t
 *
 * @param q leg joint position
 * @param p leg motor position
 * @param c leg motor current
 */
Eigen::VectorXd AnkleSolver::motor_to_joint_torque_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[5], p6 = p[4];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[11], p12 = p[10];
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];
    double i1 = c[0], i2 = c[1], i3 = c[2], i4 = c[3], i5 = c[5], i6 = c[4];
    double i7 = c[6], i8 = c[7], i9 = c[8], i10 = c[9], i11 = c[11], i12 = c[10];

    // left
    double t1 = i1;
    double t2 = i2;
    double t3 = i3;
    double t4 = i4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                0., -y_lleq * sq6 - z_lleq * cq6,
                -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                0., -y_lreq * sq6 - z_lreq * cq6,
                -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << -x_lltd * sp5 + z_lltd * cp5, 0,
                0, 0,
                -x_lltd * cp5 - z_lltd * sp5, 0,
                0, -x_lrtd * sp6 + z_lrtd * cp6,
                0, 0,
                0, -x_lrtd * cp6 - z_lrtd * sp6;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5,
                -y_llbar + y_lleq * cq6 - z_lleq * sq6,
                -x_lleq * sq5 + x_lltd * sp5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp5 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6,
                -y_lrbar + y_lreq * cq6 - z_lreq * sq6,
                -x_lreq * sq5 + x_lrtd * sp6 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp6 + z_pitch;

    Eigen::MatrixXd JAk_Act = (Jxx_l_Td * JAct_p_WTd).inverse() * (Jxx_l_Td * JAk_p_WEq);

    Eigen::Vector2d tau_Act;
    tau_Act << i5, i6;

    Eigen::Vector2d tau_Ak = JAk_Act.transpose() * tau_Act;
    double t5 = tau_Ak[0];
    double t6 = tau_Ak[1];

    // right
    double t7 = i7;
    double t8 = i8;
    double t9 = i9;
    double t10 = i10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                    0, -y_rleq * sq12 - z_rleq * cq12,
                    -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                    -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                    0, -y_rreq * sq12 - z_rreq * cq12,
                    -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << -x_rltd * sp12 + z_rltd * cp12, 0,
                        0, 0,
                        -x_rltd * cp12 - z_rltd * sp12, 0,
                        0, -x_rrtd * sp11 + z_rrtd * cp11,
                        0, 0,
                        0, -x_rrtd * cp11 - z_rrtd * sp11;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12,
                    -y_rlbar + y_rleq * cq12 - z_rleq * sq12,
                    -x_rleq * sq11 + x_rltd * sp12 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp12 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11,
                    -y_rrbar + y_rreq * cq12 - z_rreq * sq12,
                    -x_rreq * sq11 + x_rrtd * sp11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp11 + z_pitch;

    Eigen::MatrixXd JAk_Act_right = (Jxx_l_Td_right * JAct_p_WTd_right).inverse() * (Jxx_l_Td_right * JAk_p_WEq_right);

    Eigen::Vector2d tau_Act_right;
    tau_Act_right << i12, i11;

    Eigen::Vector2d tau_Ak_right = JAk_Act_right.transpose() * tau_Act_right;
    double t11 = tau_Ak_right[0];
    double t12 = tau_Ak_right[1];

    Eigen::VectorXd result(12);
    result << t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
    return result;
}
/**
 * @brief motor_v => foot_v
 *
 * @param q leg joint position
 * @param p leg motor position
 * @param v leg motor velocity
 */
Eigen::VectorXd AnkleSolver::motor_to_joint_velocity_s2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v)
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[5], p6 = p[4];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[11], p12 = p[10];
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    double q7 = q[6], q8 = q[7], q9 = q[8], q10 = q[9], q11 = q[10], q12 = q[11];
    double v1 = v[0], v2 = v[1], v3 = v[2], v4 = v[3], v5 = v[5], v6 = v[4];
    double v7 = v[6], v8 = v[7], v9 = v[8], v10 = v[9], v11 = v[11], v12 = v[10];

    // left
    double dq1 = v1;
    double dq2 = v2;
    double dq3 = v3;
    double dq4 = v4;

    double cq5 = std::cos(q5);
    double sq5 = std::sin(q5);
    double cq6 = std::cos(q6);
    double sq6 = std::sin(q6);
    double cp5 = std::cos(p5);
    double sp5 = std::sin(p5);
    double cp6 = std::cos(p6);
    double sp6 = std::sin(p6);

    Eigen::MatrixXd JAk_p_WEq(6, 2);
    JAk_p_WEq << -x_lleq * sq5 + y_lleq * sq6 * cq5 + z_lleq * cq5 * cq6, y_lleq * sq5 * cq6 - z_lleq * sq5 * sq6,
                0., -y_lleq * sq6 - z_lleq * cq6,
                -x_lleq * cq5 - y_lleq * sq5 * sq6 - z_lleq * sq5 * cq6, y_lleq * cq5 * cq6 - z_lleq * sq6 * cq5,
                -x_lreq * sq5 + y_lreq * sq6 * cq5 + z_lreq * cq5 * cq6, y_lreq * sq5 * cq6 - z_lreq * sq5 * sq6,
                0., -y_lreq * sq6 - z_lreq * cq6,
                -x_lreq * cq5 - y_lreq * sq5 * sq6 - z_lreq * sq5 * cq6, y_lreq * cq5 * cq6 - z_lreq * sq6 * cq5;

    Eigen::MatrixXd JAct_p_WTd(6, 2);
    JAct_p_WTd << -x_lltd * sp5 + z_lltd * cp5, 0,
                0, 0,
                -x_lltd * cp5 - z_lltd * sp5, 0,
                0, -x_lrtd * sp6 + z_lrtd * cp6,
                0, 0,
                0, -x_lrtd * cp6 - z_lrtd * sp6;

    Eigen::MatrixXd Jxx_l_Td(2, 6);
    Jxx_l_Td << x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5,
                -y_llbar + y_lleq * cq6 - z_lleq * sq6,
                -x_lleq * sq5 + x_lltd * sp5 + y_lleq * sq6 * cq5 - z_llbar + z_lleq * cq5 * cq6 - z_lltd * cp5 + z_pitch,
                0, 0, 0,
                0, 0, 0,
                x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6,
                -y_lrbar + y_lreq * cq6 - z_lreq * sq6,
                -x_lreq * sq5 + x_lrtd * sp6 + y_lreq * sq6 * cq5 - z_lrbar + z_lreq * cq5 * cq6 - z_lrtd * cp6 + z_pitch;

    Eigen::Vector2d ActDt;
    ActDt << v5, v6;

    Eigen::Vector2d AkDt = (Jxx_l_Td * JAk_p_WEq).colPivHouseholderQr().solve(Jxx_l_Td * JAct_p_WTd * ActDt);
    double dq5 = AkDt[0];
    double dq6 = AkDt[1];

    // right
    double dq7 = v7;
    double dq8 = v8;
    double dq9 = v9;
    double dq10 = v10;

    double cq11 = std::cos(q11);
    double sq11 = std::sin(q11);
    double cq12 = std::cos(q12);
    double sq12 = std::sin(q12);
    double cp11 = std::cos(p11);
    double sp11 = std::sin(p11);
    double cp12 = std::cos(p12);
    double sp12 = std::sin(p12);

    Eigen::MatrixXd JAk_p_WEq_right(6, 2);
    JAk_p_WEq_right << -x_rleq * sq11 + y_rleq * sq12 * cq11 + z_rleq * cq11 * cq12, y_rleq * sq11 * cq12 - z_rleq * sq11 * sq12,
                    0, -y_rleq * sq12 - z_rleq * cq12,
                    -x_rleq * cq11 - y_rleq * sq11 * sq12 - z_rleq * sq11 * cq12, y_rleq * cq11 * cq12 - z_rleq * sq12 * cq11,
                    -x_rreq * sq11 + y_rreq * sq12 * cq11 + z_rreq * cq11 * cq12, y_rreq * sq11 * cq12 - z_rreq * sq11 * sq12,
                    0, -y_rreq * sq12 - z_rreq * cq12,
                    -x_rreq * cq11 - y_rreq * sq11 * sq12 - z_rreq * sq11 * cq12, y_rreq * cq11 * cq12 - z_rreq * sq12 * cq11;

    Eigen::MatrixXd JAct_p_WTd_right(6, 2);
    JAct_p_WTd_right << -x_rltd * sp12 + z_rltd * cp12, 0,
                        0, 0,
                        -x_rltd * cp12 - z_rltd * sp12, 0,
                        0, -x_rrtd * sp11 + z_rrtd * cp11,
                        0, 0,
                        0, -x_rrtd * cp11 - z_rrtd * sp11;

    Eigen::MatrixXd Jxx_l_Td_right(2, 6);
    Jxx_l_Td_right << x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12,
                    -y_rlbar + y_rleq * cq12 - z_rleq * sq12,
                    -x_rleq * sq11 + x_rltd * sp12 + y_rleq * sq12 * cq11 - z_rlbar + z_rleq * cq11 * cq12 - z_rltd * cp12 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11,
                    -y_rrbar + y_rreq * cq12 - z_rreq * sq12,
                    -x_rreq * sq11 + x_rrtd * sp11 + y_rreq * sq12 * cq11 - z_rrbar + z_rreq * cq11 * cq12 - z_rrtd * cp11 + z_pitch;

    Eigen::Vector2d ActDt_right;
    ActDt_right << v12, v11;

    Eigen::Vector2d AkDt_right = (Jxx_l_Td_right * JAk_p_WEq_right).colPivHouseholderQr().solve(Jxx_l_Td_right * JAct_p_WTd_right * ActDt_right);
    double dq11 = AkDt_right[0];
    double dq12 = AkDt_right[1];

    Eigen::VectorXd result(12);
    result << dq1, dq2, dq3, dq4, dq5, dq6, dq7, dq8, dq9, dq10, dq11, dq12;
    return result;
}
/**
 * @brief motor_q => foot_q
 *
 * @param p leg motor position
 */
Eigen::VectorXd AnkleSolver::motor_to_joint_position_s2_(const Eigen::VectorXd& p) 
{
    double z_pitch = config[0];
    double x_lleq = config[1]; double y_lleq = config[2];  double z_lleq = config[3];
    double x_lreq = config[4]; double y_lreq = config[5]; double z_lreq = config[6];
    double y_llbar = config[7];   double z_llbar = config[8];
    double y_lrbar = config[9];  double z_lrbar = config[10];
    double x_lltd = config[11]; double z_lltd = config[12];
    double x_lrtd = config[13]; double z_lrtd = config[14];
    double l_lltd = config[15];
    double l_lrtd = config[16];
    double l_llbar = config[17];
    double l_lrbar = config[18];

    double x_rleq = config[19]; double y_rleq = config[20];  double z_rleq = config[21];
    double x_rreq = config[22]; double y_rreq = config[23]; double z_rreq = config[24];
    double y_rlbar = config[25];   double z_rlbar = config[26];
    double y_rrbar = config[27];  double z_rrbar = config[28];
    double x_rltd = config[29]; double z_rltd = config[30];
    double x_rrtd = config[31]; double z_rrtd = config[32];
    double l_rltd = config[33];
    double l_rrtd = config[34];
    double l_rlbar = config[35];
    double l_rrbar = config[36];

    double p1 = p[0], p2 = p[1], p3 = p[2], p4 = p[3], p5 = p[5], p6 = p[4];
    double p7 = p[6], p8 = p[7], p9 = p[8], p10 = p[9], p11 = p[11], p12 = p[10];

    // left
    double q1 = p1;
    double q2 = p2;
    double q3 = p3;
    double q4 = p4;

    double q5 = 0.0, q6 = 0.0;
    for (int i = 0; i < N_ITER; ++i) {
        double q50 = q5, q60 = q6;
        double cq5 = std::cos(q50), sq5 = std::sin(q50);
        double cq6 = std::cos(q60), sq6 = std::sin(q60);
        double cp5 = std::cos(p5), sp5 = std::sin(p5);
        double cp6 = std::cos(p6), sp6 = std::sin(p6);

        double x_LltendonEq_W = x_lleq * cq5 - x_lltd * cp5 + y_lleq * sq5 * sq6 + z_lleq * sq5 * cq6 - z_lltd * sp5;
        double y_LltendonEq_W = -y_llbar + y_lleq * cq6 - z_lleq * sq6;
        double z_LltendonEq_W = -std::sqrt(l_lltd * l_lltd - x_LltendonEq_W * x_LltendonEq_W - y_LltendonEq_W * y_LltendonEq_W);
        double z_WLltendon = -x_lltd * sp5 + z_llbar + z_lltd * cp5;
        double z_PitchLleq_W = z_LltendonEq_W - (z_pitch - z_WLltendon);
        double x_LrtendonEq_W = x_lreq * cq5 - x_lrtd * cp6 + y_lreq * sq5 * sq6 + z_lreq * sq5 * cq6 - z_lrtd * sp6;
        double y_LrtendonEq_W = -y_lrbar + y_lreq * cq6 - z_lreq * sq6;
        double z_LrtendonEq_W = -std::sqrt(l_lrtd * l_lrtd - x_LrtendonEq_W * x_LrtendonEq_W - y_LrtendonEq_W * y_LrtendonEq_W);
        double z_WLrtendon = -x_lrtd * sp6 + z_lrbar + z_lrtd * cp6;
        double z_PitchLreq_W = z_LrtendonEq_W - (z_pitch - z_WLrtendon);
        double z_PitchLtoe_W = (z_PitchLleq_W + z_PitchLreq_W) / 2.0;

        double x = x_lleq;
        double y = y_lleq;
        double z = z_lleq;
        double avg = z_PitchLtoe_W;
        double dev = (z_PitchLleq_W - z_PitchLreq_W) / 2.0;
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        double avg2 = avg * avg;
        double dev2 = dev * dev;
        double delta = avg2 * x2 - (x2 + z2) * (avg2 + z2 * dev2 / y2 - z2);
        if (delta < 0.0) {
            delta = 0.0;
        }
        double s_1 = (-avg * x - std::sqrt(delta)) / (x2 + z2);
        q5 = std::asin(s_1);
        double z_LrLleq_W = z_PitchLleq_W - z_PitchLreq_W;
        double z_LrLleq_Pitch = z_LrLleq_W / std::cos(q5);
        q6 = std::asin(z_LrLleq_Pitch / (y_lleq - y_lreq));
        if ((std::abs(q5 - q50) + std::abs(q6 - q60)) < 1e-6) {
            break;
        }
    }

    // right
    double q7 = p7;
    double q8 = p8;
    double q9 = p9;
    double q10 = p10;

    double q11 = 0.0, q12 = 0.0;
    for (int i = 0; i < N_ITER; ++i) {
        double q110 = q11, q120 = q12;
        double cq11 = std::cos(q110), sq11 = std::sin(q110);
        double cq12 = std::cos(q120), sq12 = std::sin(q120);
        double cp12 = std::cos(p12), sp12 = std::sin(p12);
        double cp11 = std::cos(p11), sp11 = std::sin(p11);

        double x_RltendonEq_W = x_rleq * cq11 - x_rltd * cp12 + y_rleq * sq11 * sq12 + z_rleq * sq11 * cq12 - z_rltd * sp12;
        double y_RltendonEq_W = -y_rlbar + y_rleq * cq12 - z_rleq * sq12;
        double z_RltendonEq_W = -std::sqrt(l_rltd * l_rltd - x_RltendonEq_W * x_RltendonEq_W - y_RltendonEq_W * y_RltendonEq_W);
        double z_WRltendon = -x_rltd * sp12 + z_rlbar + z_rltd * cp12;
        double z_PitchRleq_W = z_RltendonEq_W - (z_pitch - z_WRltendon);
        double x_RrtendonEq_W = x_rreq * cq11 - x_rrtd * cp11 + y_rreq * sq11 * sq12 + z_rreq * sq11 * cq12 - z_rrtd * sp11;
        double y_RrtendonEq_W = -y_rrbar + y_rreq * cq12 - z_rreq * sq12;
        double z_RrtendonEq_W = -std::sqrt(l_rrtd * l_rrtd - x_RrtendonEq_W * x_RrtendonEq_W - y_RrtendonEq_W * y_RrtendonEq_W);
        double z_WRrtendon = -x_rrtd * sp11 + z_rrbar + z_rrtd * cp11;
        double z_PitchRreq_W = z_RrtendonEq_W - (z_pitch - z_WRrtendon);
        double z_PitchLtoe_W = (z_PitchRleq_W + z_PitchRreq_W) / 2.0;

        double x = x_rleq;
        double y = y_rleq;
        double z = z_rleq;
        double avg = z_PitchLtoe_W;
        double dev = (z_PitchRleq_W - z_PitchRreq_W) / 2.0;
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        double avg2 = avg * avg;
        double dev2 = dev * dev;
        double delta = avg2 * x2 - (x2 + z2) * (avg2 + z2 * dev2 / y2 - z2);
        if (delta < 0.0) {
            delta = 0.0;
        }
        double s_1 = (-avg * x - std::sqrt(delta)) / (x2 + z2);
        q11 = std::asin(s_1);
        double z_RrRleq_W = z_PitchRleq_W - z_PitchRreq_W;
        double z_RrRleq_Pitch = z_RrRleq_W / std::cos(q11);
        q12 = std::asin(z_RrRleq_Pitch / (y_rleq - y_rreq));
        if ((std::abs(q11 - q110) + std::abs(q12 - q120)) < 1e-6) {
            break;
        }
}

Eigen::VectorXd result(12);
result << q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12;
return result;
}

// ============================================================================
// S2GEN_2 版本实现：使用 roban_ankle_solver 的方法
// ============================================================================

Eigen::VectorXd AnkleSolver::joint_to_motor_position_s2_2_(const Eigen::VectorXd& q)
{
    Eigen::VectorXd result(12);
    
    // 提取非脚踝关节（直接传递）
    result[0] = q[0];  // 左髋
    result[1] = q[1];
    result[2] = q[2];
    result[3] = q[3];  // 左膝
    result[6] = q[6];  // 右髋
    result[7] = q[7];
    result[8] = q[8];
    result[9] = q[9];  // 右膝
    
    // 提取脚踝关节角度：左 [q4, q5]，右 [q10, q11]
    Eigen::Vector4d q_ankle;
    q_ankle << q[4], q[5], q[10], q[11];
    
    // 使用 roban_ankle_solver 进行转换
    if (!roban_solver_) {
        throw std::runtime_error("roban_solver_ not initialized");
    }
    Eigen::Vector4d p_ankle = roban_solver_->joint_to_motor_position(q_ankle);
    
    result[4] = p_ankle[1];
    result[5] = p_ankle[0];
    result[10] = p_ankle[2];
    result[11] = p_ankle[3];
    
    return result;
}

Eigen::VectorXd AnkleSolver::motor_to_joint_position_s2_2_(const Eigen::VectorXd& p)
{
    Eigen::VectorXd result(12);
    
    // 提取非脚踝关节（直接传递）
    result[0] = p[0];  // 左髋
    result[1] = p[1];
    result[2] = p[2];
    result[3] = p[3];  // 左膝
    result[6] = p[6];  // 右髋
    result[7] = p[7];
    result[8] = p[8];
    result[9] = p[9];  // 右膝
    
    Eigen::Vector4d p_ankle;
    p_ankle << p[5], p[4], p[10], p[11];
    
    // 使用 roban_ankle_solver 进行转换
    if (!roban_solver_) {
        throw std::runtime_error("roban_solver_ not initialized");
    }
    Eigen::Vector4d q_ankle = roban_solver_->motor_to_joint_position(p_ankle);
    // q_ankle 格式: [左pitch, 左roll, 右pitch, 右roll]
    
    // 将结果放回 12 维向量，并应用限位
    // 左脚pitch限位
    result[4] = std::max(std::min(q_ankle[0], ankle_pitch_limits_[1]), ankle_pitch_limits_[0]);   // 左 pitch
    result[5] = std::max(std::min(q_ankle[1], ankle_roll_limits_[1]), ankle_roll_limits_[0]);   // 左 roll
    // 右脚pitch限位（与左脚使用相同限位，因为现在使用全部正向坐标系）
    result[10] = std::max(std::min(q_ankle[2], ankle_pitch_limits_[1]), ankle_pitch_limits_[0]);  // 右 pitch
    result[11] = std::max(std::min(q_ankle[3], ankle_roll_limits_[1]), ankle_roll_limits_[0]);  // 右 roll
    
    return result;
}

Eigen::VectorXd AnkleSolver::joint_to_motor_velocity_s2_2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp)
{
    Eigen::VectorXd result(12);
    
    // 提取非脚踝关节（直接传递）
    result[0] = dp[0];
    result[1] = dp[1];
    result[2] = dp[2];
    result[3] = dp[3];
    result[6] = dp[6];
    result[7] = dp[7];
    result[8] = dp[8];
    result[9] = dp[9];
    
    // 提取脚踝关节角度和速度
    Eigen::Vector4d q_ankle;
    q_ankle << q[4], q[5], q[10], q[11];
    Eigen::Vector4d p_ankle;
    p_ankle << p[5], p[4], p[10], p[11];
    Eigen::Vector4d dq_ankle;
    dq_ankle << dp[4], dp[5], dp[10], dp[11];
    
    // 使用 roban_ankle_solver 进行转换
    if (!roban_solver_) {
        throw std::runtime_error("roban_solver_ not initialized");
    }
    Eigen::Vector4d dp_ankle = roban_solver_->joint_to_motor_velocity(q_ankle, p_ankle, dq_ankle);
    
    result[4] = dp_ankle[1];
    result[5] = dp_ankle[0];
    result[10] = dp_ankle[2];
    result[11] = dp_ankle[3];
    
    return result;
}

Eigen::VectorXd AnkleSolver::motor_to_joint_velocity_s2_2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& v)
{
    Eigen::VectorXd result(12);
    
    // 提取非脚踝关节（直接传递）
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
    result[3] = v[3];
    result[6] = v[6];
    result[7] = v[7];
    result[8] = v[8];
    result[9] = v[9];
    
    // 提取脚踝关节角度和执行器速度
    Eigen::Vector4d q_ankle;
    q_ankle << q[4], q[5], q[10], q[11];
    Eigen::Vector4d p_ankle;
    p_ankle << p[5], p[4], p[10], p[11];
    Eigen::Vector4d dp_ankle;
    dp_ankle << v[5], v[4], v[10], v[11];
    
    // 使用 roban_ankle_solver 进行转换
    if (!roban_solver_) {
        throw std::runtime_error("roban_solver_ not initialized");
    }
    Eigen::Vector4d dq_ankle = roban_solver_->motor_to_joint_velocity(q_ankle, p_ankle, dp_ankle);
    // dq_ankle 格式: [左pitch速度, 左roll速度, 右pitch速度, 右roll速度]
    
    // 将结果放回 12 维向量
    result[4] = dq_ankle[0];   // 左 pitch
    result[5] = dq_ankle[1];   // 左 roll
    result[10] = dq_ankle[2];  // 右 pitch
    result[11] = dq_ankle[3];  // 右 roll
    
    return result;
}

Eigen::VectorXd AnkleSolver::joint_to_motor_current_s2_2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& t)
{
    Eigen::VectorXd result(12);
    
    // 提取非脚踝关节（直接传递）
    result[0] = t[0];
    result[1] = t[1];
    result[2] = t[2];
    result[3] = t[3];
    result[6] = t[6];
    result[7] = t[7];
    result[8] = t[8];
    result[9] = t[9];
    
    // 提取脚踝关节角度和力矩
    Eigen::Vector4d q_ankle;
    q_ankle << q[4], q[5], q[10], q[11];
    // S2GEN 格式：p[4]=左lrbar, p[5]=左llbar, p[10]=右rlbar, p[11]=右rrbar
    Eigen::Vector4d p_ankle;
    p_ankle << p[5], p[4], p[10], p[11];  // [左llbar, 左lrbar, 右llbar(rlbar), 右lrbar(rrbar)]
    Eigen::Vector4d tau_ankle;
    tau_ankle << t[4], t[5], t[10], t[11];
    
    // 使用 roban_ankle_solver 进行转换
    if (!roban_solver_) {
        throw std::runtime_error("roban_solver_ not initialized");
    }
    Eigen::Vector4d i_ankle = roban_solver_->joint_to_motor_current(q_ankle, p_ankle, tau_ankle);
    result[4] = i_ankle[1];
    result[5] = i_ankle[0];
    result[10] = i_ankle[2];
    result[11] = i_ankle[3];
    
    return result;
}

Eigen::VectorXd AnkleSolver::motor_to_joint_torque_s2_2_(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& c)
{
    Eigen::VectorXd result(12);
    
    // 提取非脚踝关节（直接传递）
    result[0] = c[0];
    result[1] = c[1];
    result[2] = c[2];
    result[3] = c[3];
    result[6] = c[6];
    result[7] = c[7];
    result[8] = c[8];
    result[9] = c[9];
    
    // 提取脚踝关节角度和执行器电流
    Eigen::Vector4d q_ankle;
    q_ankle << q[4], q[5], q[10], q[11];
    Eigen::Vector4d p_ankle;
    p_ankle << p[5], p[4], p[10], p[11];
    Eigen::Vector4d i_ankle;
    i_ankle << c[5], c[4], c[10], c[11];
    
    // 使用 roban_ankle_solver 进行转换
    if (!roban_solver_) {
        throw std::runtime_error("roban_solver_ not initialized");
    }
    Eigen::Vector4d tau_ankle = roban_solver_->motor_to_joint_torque(q_ankle, p_ankle, i_ankle);
    // tau_ankle 格式: [左pitch力矩, 左roll力矩, 右pitch力矩, 右roll力矩]
    
    // 将结果放回 12 维向量
    result[4] = tau_ankle[0];   // 左 pitch
    result[5] = tau_ankle[1];   // 左 roll
    result[10] = tau_ankle[2];  // 右 pitch
    result[11] = tau_ankle[3];  // 右 roll
    
    return result;
}
