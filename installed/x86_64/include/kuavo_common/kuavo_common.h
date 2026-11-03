#pragma once

#ifndef KUAVO_COMMON_H
#define KUAVO_COMMON_H

#include "kuavo_common/common/kuavo_settings.h"
#include "kuavo_common/common/common.h"
#include "kuavo_common/common/utils.h"
#include "kuavo_common/common/json_config_reader.hpp"
#include "kuavo_common/common/seat_config.h"

namespace HighlyDynamic
{
using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
class KuavoCommon
    {
    public:
        static KuavoCommon &getInstance(RobotVersion rb_version, const std::string& kuavo_assets_path);
        static KuavoCommon *getInstancePtr(RobotVersion rb_version, const std::string& kuavo_assets_path);

        virtual ~KuavoCommon();

        KuavoCommon(const KuavoCommon &) = delete;
        KuavoCommon &operator=(const KuavoCommon &) = delete;
        inline const KuavoSettings &getKuavoSettings() const { return kuavo_settings_; }
        JSONConfigReader *getRobotConfig() const { return robot_config_; }
        /** 座椅共享配置（v5 专有，kuavo.json 零变更） */
        const kuavo_common::SeatConfig &getSeatConfig() const { return seat_config_; }
        RobotVersion getRobotVersion() const { return rb_version_; }
    private:
        KuavoCommon(RobotVersion rb_version, const std::string& kuavo_assets_path);
        static std::shared_ptr<KuavoCommon> instance;
        RobotVersion rb_version_;

    private:
        KuavoSettings kuavo_settings_;
        JSONConfigReader *robot_config_;
        kuavo_common::SeatConfig seat_config_;
    };

}
#endif