#include "kuavo_monitor/kuavo_monitor.h"
#include "kuavo_monitor/package_path.h"

#include <stdlib.h>
#include <yaml-cpp/yaml.h>

namespace HighlyDynamic
{
    KuavoMonitor::KuavoMonitor(ros::NodeHandle &nh, const std::string &taskfile, double wbc_frequency_des)
        : nh_(nh),
          warning_threshold_(3.0),
          error_threshold_(1.5),
          total_num_(10),
          recent_num_(3),
          mpc_frequency_des_(100.0),
          wbc_frequency_des_(wbc_frequency_des),
          mpc_frequency_bias_percent_(20.0),
          wbc_frequency_bias_percent_(20.0)
        //   mpc_policy_pub_(nh)
    {
        loadParams(taskfile, true);
        mpc_frequency_analyzer_ = DataAnalyzer(total_num_, recent_num_, "MPC Frequency", print_warning_);
        mpc_time_cost_analyzer_ = DataAnalyzer(total_num_, recent_num_, "MPC Time-Cost", print_warning_);
        wbc_frequency_analyzer_ = DataAnalyzer(total_num_, recent_num_, "WBC Frequency", print_warning_);
        wbc_time_cost_analyzer_ = DataAnalyzer(total_num_, recent_num_, "WBC Time-Cost", print_warning_);

        // Subscribers
        mpc_frequency_sub_ = nh_.subscribe("/monitor/frequency/mpc", 1, &KuavoMonitor::mpcFrequencyCallback, this);
        mpc_time_cost_sub_ = nh_.subscribe("/monitor/time_cost/mpc", 1, &KuavoMonitor::mpcTimecostCallback, this);
        wbc_frequency_sub_ = nh_.subscribe("/monitor/frequency/wbc", 1, &KuavoMonitor::wbcFrequencyCallback, this);
        wbc_time_cost_sub_ = nh_.subscribe("/monitor/time_cost/wbc", 1, &KuavoMonitor::wbcTimecostCallback, this);

        // mpc_policy_pub_.run();
    }

    void KuavoMonitor::run()
    {
        const std::string path = getPath();
        // std::string command = "bash " + path + "/script/monitor_NUC.sh";
        // int result = system(command.c_str());
        YAML::Node config = YAML::LoadFile(path + "/cfg/cfg.yaml");

        ros::Rate rate(config["monitor_rate"].as<int>());
        auto node_list = config["node_list"].as<std::vector<std::string>>();

        int robot_type = 2;
        if (!nh_.getParam("/robot_type", robot_type))
        {
            std::string robot_type_str;
            if (nh_.getParam("/robot_type", robot_type_str))
            {
                try
                {
                    robot_type = std::stoi(robot_type_str);
                }
                catch (...)
                {
                    robot_type = 2;
                }
            }
        }

        while(ros::ok())
        {
          if (robot_type != 1)
          {
            for(auto node_name : node_list)
              if(!doesNodeExist(node_name))
                ROS_ERROR("Node %s is not running", node_name.c_str());
          }
          ros::spinOnce();
          rate.sleep();
        }
    }

    void KuavoMonitor::mpcFrequencyCallback(const std_msgs::Float64 &msg)
    {
        mpc_frequency_analyzer_.addData(msg.data);
        mpc_frequency_analyzer_.analyze(mpc_frequency_des_ * (1.0 - mpc_frequency_bias_percent_ / 100.0), mpc_frequency_des_ * (1.0 + mpc_frequency_bias_percent_ / 100.0));
        // mpc_frequency_analyzer_.analyzeVariance(warning_threshold_, error_threshold_);
    }

    void KuavoMonitor::mpcTimecostCallback(const std_msgs::Float64 &msg)
    {
        mpc_time_cost_analyzer_.addData(msg.data);
        mpc_time_cost_analyzer_.analyzeVariance(warning_threshold_, error_threshold_);
    }

    void KuavoMonitor::wbcFrequencyCallback(const std_msgs::Float64 &msg)
    {
        wbc_frequency_analyzer_.addData(msg.data);
        wbc_frequency_analyzer_.analyze(wbc_frequency_des_ * (1.0 - wbc_frequency_bias_percent_ / 100.0), wbc_frequency_des_ * (1.0 + wbc_frequency_bias_percent_ / 100.0));
        // wbc_frequency_analyzer_.analyzeVariance(warning_threshold_, error_threshold_);
    }

    void KuavoMonitor::wbcTimecostCallback(const std_msgs::Float64 &msg)
    {
        wbc_time_cost_analyzer_.addData(msg.data);
        wbc_time_cost_analyzer_.analyzeVariance(warning_threshold_, error_threshold_);
    }

    void KuavoMonitor::loadParams(const std::string &filename, bool verbose)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);
        std::string prefix = "MonitorParams.";

        if (verbose)
        {
            std::cerr << "\n #### Monitor Settings: ";
            std::cerr << "\n #### =============================================================================\n";
        }
        ocs2::loadData::loadPtreeValue(pt, total_num_, prefix + "totalNum", verbose);
        ocs2::loadData::loadPtreeValue(pt, recent_num_, prefix + "recentNum", verbose);
        ocs2::loadData::loadPtreeValue(pt, warning_threshold_, prefix + "warningThreshold", verbose);
        ocs2::loadData::loadPtreeValue(pt, error_threshold_, prefix + "errorThreshold", verbose);
        ocs2::loadData::loadPtreeValue(pt, mpc_frequency_bias_percent_, prefix + "mpcFrequencyBiasPercent", verbose);
        ocs2::loadData::loadPtreeValue(pt, wbc_frequency_bias_percent_, prefix + "wbcFrequencyBiasPercent", verbose);
        ocs2::loadData::loadPtreeValue(pt, print_warning_, prefix + "printWarning", verbose);

        prefix = "mpc.";
        ocs2::loadData::loadPtreeValue(pt, mpc_frequency_des_, prefix + "mpcDesiredFrequency", verbose);

        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
        }
    }

    bool KuavoMonitor::doesNodeExist(const std::string& node_name) {
      // 获取当前活跃的节点列表
      ros::V_string node_list;
      if (ros::master::getNodes(node_list))
      {
        for (const auto& node : node_list)
        {
          // std::cout << "Node: " << node << std::endl;
          if (node == node_name)
              return true; // 节点存在
        }
      }
      return false; // 节点不存在
    }
}
