#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "humanoid_interface_drake/common/json.hpp"
namespace HighlyDynamic
{
    class JSONConfigReader
    {
    private:
        nlohmann::json data_;
        std::string filename_;
        bool is_loaded_{false};

    public:
        JSONConfigReader() {}
        JSONConfigReader(const std::string &filename) : filename_(filename)
        {
            load(filename);
        }
        inline void reload() { load(filename_); }
        void load(const std::string &filename)
        {
            std::ifstream file(filename);
            if (file.is_open())
            {
                file >> data_;
            }
            else
            {
                std::cerr << "Failed to open config file: " << filename << std::endl;
            }
        }
        Eigen::VectorXd getEigenVector(const std::string &key)
        {
            if (data_.contains(key))
            {
                std::vector<double> std_vector = data_[key].get<std::vector<double>>();
                Eigen::Map<Eigen::VectorXd> eigen_vector(std_vector.data(), std_vector.size());
                return eigen_vector;
            }
            else
            {
                std::cerr << "Key not found: " << key << std::endl;
                return {};
            }
        }
        template <typename T>
        T getValue(const std::string &key)
        {
            if (data_.contains(key))
            {
                if constexpr (std::is_same<T, Eigen::Matrix<double, -1, 1>>::value)
                    return getEigenVector(key);
                else
                {
                    if constexpr (std::is_same<T, uint8_t>::value)
                    {
                        int value = data_[key].get<int>();
                        return static_cast<uint8_t>(value);
                    }
                    return data_[key].get<T>();
                }
            }
            else
            {
                std::cerr << "Key not found: " << key << std::endl;
                return T{};
            }
        }
        nlohmann::json::reference operator[](const std::string &key)
        {
            return data_[key];
        }
    };
} // namespace HighlyDynamic
