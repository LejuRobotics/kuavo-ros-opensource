#include <queue>
#include <memory>
#include <cmath>
#include "UserAppWorkpd.h"
#include "EcDemoApp.h"
#include "EcMotor.h"

// 任务基类和派生类只在.cpp定义
class TaskBase {
public:
    virtual ~TaskBase() {}
    virtual bool execute() = 0; // 返回true表示任务完成
};

class MotorInterpTask : public TaskBase {
public:
    MotorInterpTask(uint16_t id, EcMasterType* driver, double targetPos, double totalTime, double dt)
        : id_(id), driver_(driver), targetPos_(targetPos), totalTime_(totalTime), dt_(dt) {}
    bool execute() override {
        return motorMoveRelativeInterpStep(id_, driver_, targetPos_, totalTime_, dt_);
    }
private:
    uint16_t id_;
    EcMasterType* driver_;
    double targetPos_;
    double totalTime_;
    double dt_;
};

class MotorSinTask : public TaskBase {
public:
    MotorSinTask(const uint16_t* ids, uint32_t num, const MotorCspSinParam_t& param)
        : ids_(ids), num_(num), param_(param) {}
    bool execute() override {
        return workpdMotorCspSin(ids_, num_, &param_);
    }
private:
    const uint16_t* ids_;
    uint32_t num_;
    MotorCspSinParam_t param_;
};

class MotorPositionControlTask : public TaskBase {
public:
    MotorPositionControlTask(const uint16_t* ids, uint32_t num, const MotorPositionControlParam_t& param)
        : ids_(ids), num_(num), param_(param) {}
    bool execute() override {
        return workpdMotorPositionControl(ids_, num_, &param_);
    }
private:
    const uint16_t* ids_;
    uint32_t num_;
    MotorPositionControlParam_t param_;
};

class MotorActionSequenceTask : public TaskBase {
public:
    MotorActionSequenceTask(const uint16_t* ids, uint32_t num, const MotorActionSequenceParam_t& param)
        : ids_(ids), num_(num), param_(param) {}
    bool execute() override {
        return workpdMotorActionSequence(ids_, num_, &param_);
    }
private:
    const uint16_t* ids_;
    uint32_t num_;
    MotorActionSequenceParam_t param_;
};

class MotorDualLegActionSequenceTask : public TaskBase {
public:
    MotorDualLegActionSequenceTask(const uint16_t* left_ids, const uint16_t* right_ids, uint32_t num, const MotorDualLegActionSequenceParam_t& param)
        : left_ids_(left_ids), right_ids_(right_ids), num_(num), param_(param) {}
    bool execute() override {
        return workpdMotorDualLegActionSequence(left_ids_, right_ids_, num_, &param_);
    }
private:
    const uint16_t* left_ids_;
    const uint16_t* right_ids_;
    uint32_t num_;
    MotorDualLegActionSequenceParam_t param_;
};

class MotorActionFrameTask : public TaskBase {
public:
    MotorActionFrameTask(const uint16_t* ids, uint32_t num, const MotorActionFrameParam_t& param)
        : ids_(ids), num_(num), param_(param) {}
    bool execute() override {
        return workpdMotorActionFrame(ids_, num_, &param_);
    }
private:
    const uint16_t* ids_;
    uint32_t num_;
    MotorActionFrameParam_t param_;
};

class MotorMultiActionFrameTask : public TaskBase {
public:
    MotorMultiActionFrameTask(const MotorMultiActionFrameParam_t& param)
        : param_(param) {}
    bool execute() override {
        return workpdMotorMultiActionFrame(&param_);
    }
private:
    MotorMultiActionFrameParam_t param_;
};

static std::queue<std::unique_ptr<TaskBase> > g_taskQueueWorkPd;

void addMotorInterpTask(uint16_t id, EcMasterType* driver, double targetPos, double totalTime, double dt) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorInterpTask(id, driver, targetPos, totalTime, dt)));
}

void addMotorSinTask(const uint16_t* ids, uint32_t num, const MotorCspSinParam_t& param) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorSinTask(ids, num, param)));
}

void addMotorPositionControlTask(const uint16_t* ids, uint32_t num, const MotorPositionControlParam_t& param) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorPositionControlTask(ids, num, param)));
}

void addMotorActionSequenceTask(const uint16_t* ids, uint32_t num, const MotorActionSequenceParam_t& param) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorActionSequenceTask(ids, num, param)));
}

void addMotorDualLegActionSequenceTask(const uint16_t* left_ids, const uint16_t* right_ids, uint32_t num, const MotorDualLegActionSequenceParam_t& param) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorDualLegActionSequenceTask(left_ids, right_ids, num, param)));
}

void addMotorActionFrameTask(const uint16_t* ids, uint32_t num, const MotorActionFrameParam_t& param) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorActionFrameTask(ids, num, param)));
}

void addMotorMultiActionFrameTask(const MotorMultiActionFrameParam_t* param) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorMultiActionFrameTask(*param)));
}

EC_T_DWORD UserAppWorkpd(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    // printf("UserAppWorkpd called\n");
  
    // static EC_T_DWORD accTimeUsec = 0;
    // static uint32_t countTimeMs = 0;
    if (!isMotorEnable())
    {
        // accTimeUsec = 0;
        // countTimeMs = 0;
        return EC_E_ERROR;
    }

    EC_T_DWORD cycleTimeUsec = pAppContext->AppParms.dwBusCycleTimeUsec;
    // accTimeUsec += cycleTimeUsec;
    // if (accTimeUsec >= 1000)
    // {
    //     accTimeUsec -= 1000;
    //     countTimeMs++;
    // }
    // if(countTimeMs >= 1000)
    // {
    //     countTimeMs = 0;
    //     EcMasterStop();
    //     // setMotorCspSinEn(false);
    //     printf("exit\n");
        
    // }
    // printf("accTimeUsec: %d\n", accTimeUsec);
    // motorCspSinParam.dt = pAppContext->AppParms.dwBusCycleTimeUsec / 1e6;
    // printf("motorCspSinParam.dt: %f\n", motorCspSinParam.dt);

    // 处理任务队列
    if (!g_taskQueueWorkPd.empty()) {
        if (g_taskQueueWorkPd.front()->execute()) {
            g_taskQueueWorkPd.pop();
        }
    }
    else {
        // accTimeUsec = 0;
        // countTimeMs = 0;
        EcMasterStop();
    }

    return EC_E_NOERROR;
} 
