#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "kuavo_common/common/json.hpp"
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
                is_loaded_ = true;
            }
            else
            {
                std::cerr << "Failed to open config file: " << filename << std::endl;
            }
        }
        // 获取嵌套的 JSON 对象
        nlohmann::json getNestedObject(const std::string &parentKey, const std::string &childKey)
        {
            if (data_.contains(parentKey) && data_[parentKey].contains(childKey))
            {
                return data_[parentKey][childKey];
            }
            else
            {
                std::cerr << "Key not found: " << parentKey << " -> " << childKey << std::endl;
                return nullptr;
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
        // 获取嵌套的 std::vector<double> 类型的向量
        std::vector<double> getNestedStdVector(const std::string &parentKey, const std::string &childKey, const std::string &vectorKey)
        {
            auto nestedObject = getNestedObject(parentKey, childKey);
            if (nestedObject != nullptr && nestedObject.contains(vectorKey))
            {
                return nestedObject[vectorKey].get<std::vector<double>>();
            }
            else
            {
                std::cerr << "Key not found: " << parentKey << " -> " << childKey << " -> " << vectorKey << std::endl;
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
