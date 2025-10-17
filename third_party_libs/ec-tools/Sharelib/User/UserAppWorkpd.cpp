#include <queue>
#include <memory>
#include <cmath>
#include "UserAppWorkpd.h"
#include "EcDemoApp.h"
#include "EcMotor.h"

class TaskBase {
public:
    virtual ~TaskBase() {}
    virtual bool execute() = 0; // 返回true表示任务完成
};

class MotorInterpTask : public TaskBase {
public:
    MotorInterpTask(uint16_t id, double targetPos, double totalTime, double dt)
        : id_(id), targetPos_(targetPos), totalTime_(totalTime), dt_(dt) {}
    bool execute() override {
        return motorMoveRelativeInterpStep(id_, targetPos_, totalTime_, dt_);
    }
private:
    uint16_t id_;
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

class MotorMultiActionTask : public TaskBase {
public:
    MotorMultiActionTask(const uint16_t* ids, uint32_t num, const MotorMultiActionParam_t& param)
        : ids_(ids), num_(num), param_(param) {}
    bool execute() override {
        return workpdMotorMultiAction(ids_, num_, &param_);
    }
private:
    const uint16_t* ids_;
    uint32_t num_;
    MotorMultiActionParam_t param_;
};

static std::queue<std::unique_ptr<TaskBase> > g_taskQueueWorkPd;

void addMotorInterpTask(uint16_t id, double targetPos, double totalTime, double dt) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorInterpTask(id, targetPos, totalTime, dt)));
}

void addMotorSinTask(const uint16_t* ids, uint32_t num, const MotorCspSinParam_t& param) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorSinTask(ids, num, param)));
}

void addMotorMultiActionTask(const uint16_t* ids, uint32_t num, const MotorMultiActionParam_t& param) {
    g_taskQueueWorkPd.push(std::unique_ptr<TaskBase>(new MotorMultiActionTask(ids, num, param)));
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
        // printf("taskQueueWorkPd is empty\n");
        // accTimeUsec = 0;
        // countTimeMs = 0;
        EcMasterStop();
    }

    return EC_E_NOERROR;
} 
