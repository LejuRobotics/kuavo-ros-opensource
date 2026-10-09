#pragma once
#include <string>
namespace humanoid_controller
{
  /**
   * @brief 控制器大类枚举
   * 定义控制器的大类分类
   */
  enum class ControllerClass
  {
    BASE_CONTROLLER = 0,      ///< 基础控制器大类（MPC等）
    FALL_STAND_CONTROLLER,    ///< 倒地起身控制器大类
    DANCE_CONTROLLER,         ///< 舞蹈控制器大类
  };

  /**
   * @brief RL控制器类型枚举
   * 定义支持的具体RL控制器类型
   */
  enum class RLControllerType
  {
    MPC = 0,                  ///< MPC控制器
    AMP_CONTROLLER,           ///< AMP行走控制器
    FALL_STAND_CONTROLLER,    ///< 倒地起身控制器
    PERCEPTION_LOCO_CONTROLLER, ///< 感知行走控制器
    DEPTH_LOCO_CONTROLLER,
    VMP_CONTROLLER,           ///< VMP控制器
    DANCE_CONTROLLER,         ///< Dance控制器
  };


  /**
   * @brief 控制器状态枚举
   * 用于控制器内部状态管理
   */
  enum class ControllerState
  {
    INITIALIZING = 0,    ///< 初始化中
    RUNNING,         ///< 运行中
    PAUSED,          ///< 暂停
    ERROR,           ///< 错误状态
    STOPPED,         ///< 停止
  };

  /**
   * @brief 将字符串转换为ControllerClass枚举
   * @param class_str 控制器大类字符串
   * @param controller_class 输出的控制器大类（输出参数）
   * @return 如果转换成功返回true，否则返回false
   */
  inline bool stringToControllerClass(const std::string& class_str, ControllerClass& controller_class)
  {
    if (class_str == "BASE_CONTROLLER")
    {
      controller_class = ControllerClass::BASE_CONTROLLER;
      return true;
    }
    else if (class_str == "FALL_STAND_CONTROLLER")
    {
      controller_class = ControllerClass::FALL_STAND_CONTROLLER;
      return true;
    }
    else if (class_str == "DANCE_CONTROLLER")
    {
      controller_class = ControllerClass::DANCE_CONTROLLER;
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
   * @brief 将字符串转换为RLControllerType枚举
   * @param type_str 控制器类型字符串
   * @param type 输出的控制器类型（输出参数）
   * @return 如果转换成功返回true，否则返回false
   */
  inline bool stringToRLControllerType(const std::string& type_str, RLControllerType& type)
  {
    if (type_str == "MPC")
    {
      type = RLControllerType::MPC;
      return true;
    }
    else if (type_str == "AMP_CONTROLLER")
    {
      type = RLControllerType::AMP_CONTROLLER;
      return true;
    }
    else if (type_str == "FALL_STAND_CONTROLLER")
    {
      type = RLControllerType::FALL_STAND_CONTROLLER;
      return true;
    }
    else if (type_str == "PERCEPTION_LOCO_CONTROLLER")
    {
      type = RLControllerType::PERCEPTION_LOCO_CONTROLLER;
      return true;
    }
    else if (type_str == "DEPTH_LOCO_CONTROLLER")
    {
      type = RLControllerType::DEPTH_LOCO_CONTROLLER;
      return true;
    }
    else if (type_str == "VMP_CONTROLLER")
    {
      type = RLControllerType::VMP_CONTROLLER;
      return true;
    }
    else if (type_str == "DANCE_CONTROLLER")
    {
      type = RLControllerType::DANCE_CONTROLLER;
      return true;
    }
    else
    {
      return false;
    }
  }

} // namespace humanoid_controller










