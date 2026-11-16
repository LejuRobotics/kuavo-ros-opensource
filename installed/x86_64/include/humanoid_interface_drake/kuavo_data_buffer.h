#pragma once

#include <iostream>
#include "Eigen/Core"
#include "kuavo_common/common/robot_state.h"
#include "kuavo_common/common/kuavo_settings.h"
#include <deque>
#include <condition_variable>
#include <ros/ros.h>

template <typename T>
class KuavoDataBuffer
{
private:
  std::string bufferName; // 缓冲区名称
  std::deque<std::pair<double, T>> dataBuffer;
  size_t max_size; // 最大缓冲区大小
  std::mutex buffer_mutex;
  std::condition_variable cv;
  double dt_{0.001};
  double lastGetTime = 0.0;
  double last_notify_time{0.0};
  bool is_synced{false};

public:
  KuavoDataBuffer(const std::string &name = "data_buffer", size_t size = 200, double dt = 0.001) : bufferName(name), max_size(size), dt_(dt) {} // 构造函数接受缓冲区名称和最大缓冲区大小

  void addData(double timestamp, T data)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    dataBuffer.emplace_back(timestamp, data);
    if (dataBuffer.size() > max_size)
    {
      dataBuffer.pop_front();
      // if (is_synced)
      //   std::cout << "Warning: " << bufferName << " droping data, max_size:" << max_size << " " << dataBuffer.size() << std::endl;
    }
    cv.notify_all();
  }

  T getNextData()
  {
    std::unique_lock<std::mutex> lock(buffer_mutex);
    cv.wait(lock, [this]
            { return !dataBuffer.empty(); });
    if (dataBuffer.front().first - lastGetTime > 2 * dt_)
    {
      std::cout << "Warning: " << bufferName << " data is too old, dt: " << dataBuffer.front().first - lastGetTime << std::endl;
    }
    lastGetTime = dataBuffer.front().first;

    T nextData = dataBuffer.front().second;
    dataBuffer.pop_front();
    return nextData;
  }
  T getData(double timestamp) // 获取最接近timestamp的数据
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    double lastTimeDiff = std::abs(dataBuffer.back().first - timestamp); // 最后一个数据与目标时间戳的时间差
    size_t nearestIndex = dataBuffer.size() - 1;                         // 最近数据的索引，初始为最后一个数据的索引
    for (int i = dataBuffer.size() - 2; i >= 0; --i)
    {
      double timeDiff = std::abs(dataBuffer[i].first - timestamp); // 计算时间差
      if (timeDiff <= lastTimeDiff)
      {
        lastTimeDiff = timeDiff;
        nearestIndex = i;
      }
      else
      {
        // 时间差开始增加，说明已经找到最近的数据
        break;
      }
    }

    // 获取最近的数据
    T nearestData = dataBuffer[nearestIndex].second;
    if (lastTimeDiff > 0.003 && timestamp - last_notify_time > 0.3)
    {
      last_notify_time = timestamp;
      std::cout << "Warning: " << bufferName << " using nearest data, id:" << nearestIndex << ", dt:" << dataBuffer[nearestIndex].first - timestamp << std::endl;
    }
    if (dataBuffer.size() > 2)
      dataBuffer.pop_front();
    return nearestData;
  }
  T getLastData()
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (dataBuffer.size() > 2)
      dataBuffer.pop_front();
    return dataBuffer.back().second;
  }
  void waitForReady()
  {
    std::cout << "wait for " << bufferName << " to be ready" << std::endl;
    while (ros::ok() && !isReady())
    {
      ros::spinOnce();
      usleep(1000);
    }

    std::cout << bufferName << " is ready" << std::endl;
  }
  bool isReady()
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    return dataBuffer.size() >= max_size - 1;
  }
  size_t size()
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    return dataBuffer.size();
  }

  void sync(double sync_threshold = 0.002)
  {
    std::cout << "Syncing with buffer " << bufferName << std::endl;
    double thresholdTime = ros::Time::now().toSec() - dt_;
    double currentTime;

    while (ros::ok()) // wait for available time
    {
      {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        if (!dataBuffer.empty())
        {
          currentTime = ros::Time::now().toSec();
          if (std::abs(currentTime - dataBuffer.back().first) < sync_threshold || currentTime > dataBuffer.front().first)
          {
            thresholdTime = dataBuffer.back().first;
            std::cout << "Synced with buffer " << bufferName << ", threshold time: " << thresholdTime << std::endl;
            break;
          }
        }
      }
      usleep(1000); // 如果没有符合条件的数据，则等待一段时间再重复判断
    }

    // 清除过期数据
    while (!dataBuffer.empty() && dataBuffer.front().first < thresholdTime)
    {
      dataBuffer.pop_front();
    }
    is_synced = true;
    std::cout << "Synced with buffer " << bufferName << " done!" << std::endl;
  }
};

class SensorDataBuffer
{
private:
  std::deque<SensorData_t> buffer;
  size_t max_size;
  std::mutex buffer_mutex;
  double last_notify_time{0.0};

public:
  SensorDataBuffer(size_t size) : max_size(size) {}

  void addData(SensorData_t data)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffer.push_back(data);
    while (buffer.size() > max_size)
    {
      buffer.pop_front();
    }
  }
  size_t size()
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    return buffer.size();
  }
  bool isReady()
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    return buffer.size() >= max_size;
  }
  SensorData_t getData(double timestamp) // 获取最接近timestamp的数据
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    double lastTimeDiff = std::abs(buffer.back().time - timestamp); // 最后一个数据与目标时间戳的时间差
    size_t nearestIndex = buffer.size() - 1;                        // 最近数据的索引，初始为最后一个数据的索引
    for (int i = buffer.size() - 2; i >= 0; --i)
    {
      double timeDiff = std::abs(buffer[i].time - timestamp); // 计算时间差
      if (timeDiff <= lastTimeDiff)
      {
        lastTimeDiff = timeDiff;
        nearestIndex = i;
      }
      else
      {
        // 时间差开始增加，说明已经找到最近的数据
        break;
      }
    }

    // 获取最近的数据
    SensorData_t nearestData = buffer[nearestIndex];
    if (lastTimeDiff > 0.003 && timestamp - last_notify_time > 0.3)
    {
      last_notify_time = timestamp;
      std::cout << "Warning: sensor_data is not up to date, using nearest data, id:" << nearestIndex << ", dt:" << nearestData.time - timestamp << std::endl;
    }
    return nearestData;
  }
};