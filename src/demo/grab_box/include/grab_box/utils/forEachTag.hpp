#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <vector>
#include <iostream>
#include <unordered_map>

using TagStatusMap = std::unordered_map<int, int>;

class ForEachTag : public BT::DecoratorNode {
public:
    ForEachTag(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config), current_index_(0) 
        {
            
        }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<TagStatusMap>("tag_status_map"),  // 输入：动态tag状态表
            BT::InputPort<std::vector<int>>("sorted_tag_ids"),  // 输入：已排序的tag id列表
            BT::OutputPort<int>("current_tag_id")          // 输出：当前处理的tag_id
        };
    }

    BT::NodeStatus tick() override {
        if (status() == BT::NodeStatus::IDLE) {
            // 初始化：获取 tag_status_map
            if (!getInput("tag_status_map", tag_status_map_)) {
                throw BT::RuntimeError("Missing required input [tag_status_map]");
            }

            if (!getInput("sorted_tag_ids", sorted_tag_ids_)) {
                throw BT::RuntimeError("Missing required input [sorted_tag_ids]");
            }

            // // 筛选未操作的tags
            // pending_tags_.clear();
            // for (const auto& [id, status] : tag_status_map_) {
            //     if (status == 0) { // 状态为未操作
            //         pending_tags_.push_back(id);
            //     }
            // }

            // 清空pending_tags_并重新填充，确保只处理状态为0的tags
            pending_tags_.clear();
            for (const auto& tag_id : sorted_tag_ids_) {
                auto it = tag_status_map_.find(tag_id);
                if (it != tag_status_map_.end() && it->second == 0) {
                    pending_tags_.push_back(tag_id);
                }
            }


            current_index_ = 0;
        }

        // 遍历待处理的tag
        if (current_index_ < pending_tags_.size()) {
            int tag_id = pending_tags_[current_index_];
            setOutput("current_tag_id", tag_id);

            // 更新状态为运行中
            tag_status_map_[tag_id] = 1;

            // 执行子节点
            auto child_status = child_node_->executeTick();

            if (child_status == BT::NodeStatus::SUCCESS) {
                tag_status_map_[tag_id] = 2; // 更新状态为成功
                current_index_++;
                if (current_index_ < pending_tags_.size()) {
                    return BT::NodeStatus::RUNNING;
                } else {
                    return BT::NodeStatus::SUCCESS;
                }
            } else if (child_status == BT::NodeStatus::FAILURE) {
                tag_status_map_[tag_id] = 3; // 更新状态为失败
                return BT::NodeStatus::FAILURE;
            } else {
                return child_status; // 返回子节点状态
            }
        } else {
            return BT::NodeStatus::SUCCESS; // 所有tag已处理完
        }
    }

    void halt() override {
        current_index_ = 0;
        pending_tags_.clear();
        DecoratorNode::halt();
    }

private:
    TagStatusMap tag_status_map_;        // 动态tag状态表
    std::vector<int> pending_tags_;     // 未操作的tag id列表
    size_t current_index_;              // 当前处理的索引
    std::vector<int> sorted_tag_ids_;        // 已排序的tag id列表
};
