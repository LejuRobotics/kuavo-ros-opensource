#ifndef _utils_h_
#define _utils_h_

#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <filesystem>
#include <csignal>
#include <experimental/filesystem>
#include <cstdio>
#include <sstream>
#include <cstdlib>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

namespace fs = std::experimental::filesystem;

#define TIME_DIFF_MS(t0, t1) ((t1.tv_sec + t1.tv_nsec * 1e-9) - (t0.tv_sec + t0.tv_nsec * 1e-9)) * 1e3
#define TIME_DIFF(t0, t1) ((t1.tv_sec + t1.tv_nsec * 1e-9) - (t0.tv_sec + t0.tv_nsec * 1e-9))
#define LIMITING(v, min, max) ((v) > (max) ? (max) : ((v) < (min) ? (min) : (v)))

#define TO_DEGREE (180.0 / M_PI)
#define TO_RADIAN (M_PI / 180.0)

#define zeros(rows, cols) Eigen::MatrixXd::Zero(rows, cols)
#define eye(rows, cols) Eigen::MatrixXd::Identity(rows, cols)
#define ones(rows, cols) Eigen::MatrixXd::Ones(rows, cols)

std::string GetAbsolutePathFromRosPackage(const std::string &package_name, const std::string &path);

void printArrayI(const char *name, const int32_t *data, uint32_t size);
void printArrayF(const char *name, const double *data, uint32_t size);

int32_t thread_rt();
int32_t process_rt();
int32_t sched_thread(int p = 40);
int32_t sched_process(int p = 40);
std::string getUserHomeDirectory();

bool readCsvData(const char *file_name, bool skip_header, std::vector<std::vector<double>> &data);
inline double normalize_angle(double angle)
{
    bool negative = angle + M_PI < 0;
    angle = fmod(angle + M_PI, 2 * M_PI);

    return (negative) ? angle + M_PI : angle - M_PI;
}
int kbhit(void);

void print_cpu_mask(cpu_set_t cpu_mask);
int8_t get_cpu_mask(pid_t pid, cpu_set_t *mask);
int8_t set_cpu_mask(pid_t pid, cpu_set_t *mask);

class TeeRedirect
{
public:
    TeeRedirect()
    {
        initializeLog();
        start();
    }

    void start()
    {
        pipe = popen(("tee -i " + filename).c_str(), "w");
        if (pipe == nullptr)
        {
            std::cerr << "Error opening pipe to tee, coould not redirect output to file." << std::endl;
            return;
        }

        originalStdout = dup(fileno(stdout));
        originalStderr = dup(fileno(stderr));
        dup2(fileno(pipe), fileno(stdout));
        dup2(fileno(pipe), fileno(stderr));
    }

    void finish()
    {
        std::cout << "终端输出日志保存到: " << filename << std::endl;
        fflush(stdout);
        fflush(stderr);
        dup2(originalStderr, fileno(stderr));
        dup2(originalStdout, fileno(stdout));
        pclose(pipe);
    }

private:
    std::string filename;
    FILE *pipe;
    int originalStdout;
    int originalStderr;

    void initializeLog()
    {
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&timestamp), "%Y%m%d_%H%M%S");
        std::string timestampStr = ss.str();

        filename = getUserHomeDirectory() + "/.log/log_" + timestampStr + ".txt";
        std::cout << "终端输出日志保存到: " << filename << std::endl;

        fs::path filePath(filename);
        if (!filePath.parent_path().empty() && !fs::exists(filePath.parent_path()))
        {
            fs::create_directories(filePath.parent_path());
        }
    }
};
class StdoutStreamBuf : public std::streambuf
{
public:
    StdoutStreamBuf()
    {
        // 获取终端的原始streambuf
        originalCoutBuf = std::cout.rdbuf();

        // 获取当前时间戳并格式化为字符串
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&timestamp), "%Y%m%d_%H%M%S");
        std::string timestampStr = ss.str();

        filename = getUserHomeDirectory() + "/.log/log_" + timestampStr + ".txt";
        std::cout << "终端输出日志保存到: " << filename << std::endl;

        fs::path filePath(filename);
        if (!filePath.parent_path().empty() && !fs::exists(filePath.parent_path()))
        {
            fs::create_directories(filePath.parent_path());
        }

        fileStream.open(filename);
    }

    ~StdoutStreamBuf()
    {
        std::cout << "Terminal output saved to: " << filename << std::endl;
        sync();
        fileStream.close();
    }

protected:
    virtual int_type overflow(int_type c) override
    {
        if (c != EOF)
        {
            if (fileStream.is_open())
            {
                fileStream.put(c);
            }
            originalCoutBuf->sputc(c); // 写入原始buffer
        }
        return c;
    }

    virtual int sync() override
    {
        fileStream.flush();
        originalCoutBuf->pubsync(); // 刷新原始buffer
        return 0;
    }

private:
    std::string filename;
    std::ofstream fileStream;
    std::streambuf *originalCoutBuf;
};

struct PeriodicData
{
    std::deque<Eigen::Vector3d> buffer;
    int bufferSize;

    PeriodicData(int size) : bufferSize(size) {}

    void addData(const Eigen::Vector3d &data)
    {
        buffer.push_back(data);

        if (buffer.size() > bufferSize)
        {
            buffer.pop_front();
        }
    }
    bool isBufferFull() const
    {
        return buffer.size() == bufferSize;
    }

    Eigen::Vector3d computeVariance(const Eigen::Vector3d &data)
    {
        addData(data);
        return computeVariance();
    }
    // 求方差
    Eigen::Vector3d computeVariance()
    {
        int numPeriods = buffer.size();
        if (numPeriods < bufferSize)
        {
            return Eigen::Vector3d(0, 0, 0);
        }

        Eigen::Vector3d mean(0.0, 0.0, 0.0);
        for (const auto &data : buffer)
        {
            mean += data;
        }
        mean /= numPeriods;

        Eigen::Vector3d variance(0.0, 0.0, 0.0);
        for (const auto &data : buffer)
        {
            Eigen::Vector3d diff = data - mean;
            variance += diff.array().square().matrix();
        }
        variance /= numPeriods;

        return variance;
    }
};

class ThreadPool
{
public:
    ThreadPool(int num_threads, int priority) : priority_(priority)
    {
        for (int i = 0; i < num_threads; ++i)
        {
            threads_.emplace_back([this]()
                                  {
                struct sched_param params;
                params.sched_priority = priority_;
                pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
                while (!stop_) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        condition_.wait(lock, [this]() { return !tasks_.empty() || stop_; });
                        if (stop_ && tasks_.empty()) {
                            return;
                        }
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    task();
                } });
        }
    }

    ~ThreadPool()
    {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            stop_ = true;
        }
        condition_.notify_all();
        for (auto &thread : threads_)
        {
            thread.join();
        }
    }

    template <typename Func, typename... Args>
    void Submit(Func &&func, Args &&...args)
    {
        std::function<void()> task = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);
        {
            std::unique_lock<std::mutex> lock(mutex_);
            tasks_.emplace(std::move(task));
        }
        condition_.notify_one();
    }

    void Cancel()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!tasks_.empty())
        {
            tasks_.front() = nullptr;
            tasks_.pop();
        }
    }

    void SetPriority(int priority)
    {
        priority_ = priority;
        for (auto &thread : threads_)
        {
            struct sched_param params;
            params.sched_priority = priority_;
            pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &params);
        }
    }

private:
    std::vector<std::thread> threads_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stop_{false};
    int priority_;
};

class ThreadSync
{
public:
    ThreadSync(int num_threads);

    void WaitForSync();

private:
    std::mutex mtx_;
    std::condition_variable cv_;
    int num_threads_;
    int thread_count_;
};

std::string getKuavoHomePath();
std::string getKuavoOffsetFilePath();
std::vector<std::vector<double_t>> loadKuavoOffsetPosition(std::string offset_file_path);
std::string getKuavoEcMasterLicensePath();
#endif
