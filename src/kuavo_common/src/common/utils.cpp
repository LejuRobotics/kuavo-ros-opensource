#include "kuavo_common/common/utils.h"
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <gflags/gflags.h>
#include <iostream>
#include <filesystem>
#include <unistd.h>
#include <streambuf>
#include <experimental/filesystem>
#include <iomanip>
#include <chrono>
#include <pwd.h>
DECLARE_bool(log_lcm);

std::filesystem::path file_path = __FILE__;
std::filesystem::path CURRENT_SOURCE_DIR = file_path.parent_path().parent_path();
// 获取普通用户的目录
std::string getUserHomeDirectory()
{
    uid_t uid;
    char *sudoUser = getenv("SUDO_USER");

    if (sudoUser != NULL)
    {
        struct passwd *pw = getpwnam(sudoUser);
        if (pw)
        {
            return std::string(pw->pw_dir);
        }
    }
    else
    {
        uid = getuid();
        struct passwd *pw = getpwuid(uid);
        if (pw)
        {
            return std::string(pw->pw_dir);
        }
    }
    return "";
}


void printArrayI(const char *name, const int32_t *data, uint32_t size)
{
    printf("%s: [", name);
    for (uint32_t i = 0; i < size - 1; i++)
    {
        printf("%d, ", data[i]);
    }
    printf("%d]\n", data[size - 1]);
}

void printArrayF(const char *name, const double *data, uint32_t size)
{
    printf("%s: [", name);
    for (uint32_t i = 0; i < size - 1; i++)
    {
        printf("%f, ", data[i]);
    }
    printf("%f]\n", data[size - 1]);
}

int32_t thread_rt()
{
    int32_t ret;
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO); // SCHED_RR
    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0)
    {
        printf("Failed to set rt of thread %d. %s\n", int(pthread_self()), strerror(ret));
    }
    return ret;
}

int32_t process_rt()
{
    int32_t ret;
    pid_t pid = getpid();
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO); // SCHED_RR
    ret = sched_setscheduler(pid, SCHED_FIFO, &param);
    if (ret != 0)
    {
        printf("Failed to set rt of process %d. %s\n", pid, strerror(ret));
    }
    return ret;
}

int32_t sched_thread(int p)
{
    int32_t ret;
    struct sched_param param;
    param.sched_priority = p;
    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0)
    {
        printf("Failed to set thread sched %d. %s\n", int(pthread_self()), strerror(ret));
    }
    return ret;
}

int32_t sched_process(int p)
{
    int32_t ret;
    pid_t pid = getpid();
    struct sched_param param;
    param.sched_priority = p;
    ret = sched_setscheduler(pid, SCHED_FIFO, &param);
    if (ret != 0)
    {
        printf("Failed to process sched %d. %s\n", pid, strerror(ret));
    }
    return ret;
}

bool readCsvData(const char *file_name, bool skip_header, std::vector<std::vector<double>> &data)
{
    std::ifstream inFile(file_name);
    if (!inFile.is_open())
    {
        std::cout << "Error:" << __FILE__ << " in line " << __LINE__ << ", Failed to open file!\n";
        return false;
    }

    std::string line;
    if (skip_header)
    {
        std::getline(inFile, line);
    }

    while (std::getline(inFile, line))
    {
        std::vector<double> line_vec;
        std::string number;
        std::istringstream readstr(line);
        while (std::getline(readstr, number, ','))
        {
            line_vec.push_back(atof(number.c_str()));
        }
        data.push_back(line_vec);
    }
    return true;
}

int kbhit(void)
{

    fd_set rfds;
    struct timeval tv;
    int retval;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    retval = select(1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */

    if (retval == -1)
    {
        perror("select()");
        return 0;
    }
    else if (retval)
        return 1;
    /* FD_ISSET(0, &rfds) will be true. */
    else
        return 0;
    return 0;
}

void print_cpu_mask(cpu_set_t cpu_mask)
{
    unsigned char flag = 0;
    printf("Cpu affinity is ");
    for (unsigned int i = 0; i < sizeof(cpu_set_t); i++)
    {
        if (CPU_ISSET(i, &cpu_mask))
        {
            if (flag == 0)
            {
                flag = 1;
                printf("%d", i);
            }
            else
            {
                printf(",%d", i);
            }
        }
    }
    printf(".\n");
}

int8_t get_cpu_mask(pid_t pid, cpu_set_t *mask)
{
    if (sched_getaffinity(pid, sizeof(cpu_set_t), mask) == -1)
    {
        printf("Failed to get cpu affinity.\n");
        return -1;
    }
    return 0;
}

int8_t set_cpu_mask(pid_t pid, cpu_set_t *mask)
{
    if (sched_setaffinity(pid, sizeof(cpu_set_t), mask) == -1)
    {
        printf("Failed to set cpu affinity.\n");
        return -1;
    }
    return 0;
}
ThreadSync::ThreadSync(int num_threads) : num_threads_(num_threads), thread_count_(0){};

void ThreadSync::WaitForSync()
{
    std::unique_lock<std::mutex> lock(mtx_);
    thread_count_++;
    if (thread_count_ == num_threads_)
    {
        thread_count_ = 0;
        lock.unlock();
        cv_.notify_all();
    }
    else
    {
        cv_.wait(lock); // 等待条件变量
        lock.unlock();
    }
}

std::string getKuavoHomePath()
{
    uid_t uid;
    char *sudoUser = getenv("SUDO_USER");

    if (sudoUser != NULL)
    {
        struct passwd *pw = getpwnam(sudoUser);
        if (pw)
        {
            return std::string(pw->pw_dir);
        }
    }
    else
    {
        uid = getuid();
        struct passwd *pw = getpwuid(uid);
        if (pw)
        {
            return std::string(pw->pw_dir);
        }
    }
    return "";
}

std::string getKuavoEcMasterLicensePath()
{
    std::string homePath = getKuavoHomePath();
    if (!homePath.empty())
    {
        return homePath + "/.config/lejuconfig/ec_master.key";
    }
    return "";
}

std::string getKuavoOffsetFilePath()
{
    std::string homePath = getKuavoHomePath();
    if (!homePath.empty())
    {
        return homePath + "/.config/lejuconfig/offset.csv";
    }
    return "";
}

bool csvLoad(const char *file_name, bool skip_header, std::vector<std::vector<double>> &data)
{
    std::ifstream inFile(file_name);
    if (!inFile.is_open())
    {
        return false;
    }

    std::string line;
    if (skip_header)
    {
        std::getline(inFile, line);
    }

    while (std::getline(inFile, line))
    {
        std::vector<double> line_vec;
        std::string number;
        std::istringstream readstr(line);
        while (std::getline(readstr, number, ','))
        {
            line_vec.push_back(atof(number.c_str()));
        }
        data.push_back(line_vec);
    }
    return true;
}

std::vector<std::vector<double_t>> loadKuavoOffsetPosition(std::string offset_file_path)
{
    std::vector<std::vector<double_t>> offset;
    std::string offset_file = offset_file_path;
    if (offset_file.empty())
    {
        for (auto &vec : offset)
        {
            for (auto &val : vec)
            {
                val *= 4 * 360.0;
            }
        }
        return offset;
    }
    if (!csvLoad(offset_file.c_str(), false, offset))
    {
        for (auto &vec : offset)
        {
            for (auto &val : vec)
            {
                val *= 4 * 360.0;
            }
        }
        return offset;
    }
    return offset;
}
