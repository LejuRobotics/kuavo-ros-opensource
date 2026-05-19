#include <iostream>
#include <deque>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace
{
    const std::string YELLOW = "\033[33m";
    const std::string RED = "\033[31m";
    const std::string RESET = "\033[0m";
}

namespace HighlyDynamic
{
    class DataAnalyzer
    {
    public:
        DataAnalyzer() {}

        DataAnalyzer(int N, int recent_num, const std::string &data_name, bool print_warning = true)
            : N(N), recent_num_(recent_num), data_name_(data_name), print_warning_(print_warning) {}

        void addData(float data)
        {
            data_queue_.push_back(data);
            if (data_queue_.size() > N)
            {
                data_queue_.pop_front();
            }
        }

        void analyze(float lowerThreshold, float upperThreshold)
        {
            if (data_queue_.size() < N)
                return; // 数据不足N个，不进行分析
            float lastData = data_queue_.back();
            if (lastData < lowerThreshold)
                printWarning("最新的 " + data_name_ + " 数据(" + std::to_string(lastData) + ")低于下限阈值(" + std::to_string(lowerThreshold) + ") !");
            else if (lastData > upperThreshold)
                printWarning("最新的 " + data_name_ + " 数据(" + std::to_string(lastData) + ")高于上限阈值(" + std::to_string(upperThreshold) + ") !");

            bool continous_low = true;
            bool continous_high = true;
            std::for_each(data_queue_.end() - recent_num_, data_queue_.end(), [&](float data)
                          { continous_low &= (data < lowerThreshold); });
            std::for_each(data_queue_.end() - recent_num_, data_queue_.end(), [&](float data)
                          { continous_high &= (data > upperThreshold); });

        }

        void analyzeVariance(float warningThreshold, float errorThreshold)
        {
            if (data_queue_.size() < N)
            {
                return; // 数据不足N个，不进行分析
            }
            float variance = calculateVariance();
            float lastData = data_queue_.back();
            float mean = calculateMean();

            // 如果最新的一个数据明显大于平均值加上方差，则弹出警告
            if (lastData > mean + sqrt(variance) * warningThreshold)
            {
                printWarning("最新的 " + data_name_ + " 数据(" + std::to_string(lastData) + ") 与平均值(" + std::to_string(mean) + ")差异过大 !");
            }

            // 如果最新的3个数据与平均值加上方差的差异过大，则程序报错
            if (data_queue_.size() >= recent_num_)
            {
                float sumLastThree = std::accumulate(data_queue_.end() - recent_num_, data_queue_.end(), 0.0);
                if (sumLastThree / recent_num_ > mean + sqrt(variance) * errorThreshold)
                {
                    printError("连续" + std::to_string(recent_num_) + "个 " + data_name_ + " 数据均值(" + std::to_string(sumLastThree / recent_num_) + ")与平均值(" + std::to_string(mean) + ")差异过大!!!");
                }
            }
        }

    private:
        int N;
        int recent_num_;
        std::string data_name_;
        std::deque<float> data_queue_;
        bool print_warning_ = false;

        float calculateMean()
        {
            return std::accumulate(data_queue_.begin(), data_queue_.end(), 0.0) / data_queue_.size();
        }

        float calculateVariance()
        {
            float mean = calculateMean();
            float variance = 0.0;
            for (float &data : data_queue_)
            {
                variance += (data - mean) * (data - mean);
            }
            return variance / data_queue_.size();
        }

        void printWarning(const std::string &message)
        {
            if (print_warning_)
                std::cout << YELLOW << message << RESET << std::endl;
        }

        void printError(const std::string &message)
        {
            std::cout << RED << message << RESET << std::endl;
        }
    };

}