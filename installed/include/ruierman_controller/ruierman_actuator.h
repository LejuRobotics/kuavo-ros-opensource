#ifndef RUIERMAN_ACTUATOR_H
#define RUIERMAN_ACTUATOR_H

#include <Python.h>
#include <vector>
#include <thread>
#include <iostream>
static PyObject *pJoinMethod;// 用于将python线程移动到c++线程中,避免GIL占用

class RuiErManActuator
{
private:
    PyObject *pModule;
    PyObject *RuiErManActuatorClass;
    PyObject *ActuatorInstance;

    PyObject *pEnableMethod;
    PyObject *pCloseMethod;
    PyObject *pDisableMethod;
    PyObject *pSetPositionMethod;
    PyObject *pGetJointStateMethod;
    std::string pymodule_path;
    PyGILState_STATE gstate;
    std::thread pythonThread;

public:
    // 需要传入python模块所在路径
    RuiErManActuator(std::string pymodule_path = "");

    ~RuiErManActuator();

    int initialize();
    void enable();
    void disable();
    void close();
    static void join();
    void set_positions(std::vector<uint8_t> ids, std::vector<double> positions);
    std::vector<double> get_positions();
};

#endif // RUIERMAN_ACTUATOR_H
