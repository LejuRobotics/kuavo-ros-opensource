#pragma once

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/tree_node.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

namespace GrabBox
{

/**
 * @brief A custom logger that prints status changes to std::cout,
 * but ignores a specific list of node types.
 *
 * This is useful to reduce verbosity when debugging complex trees, by hiding
 * frequently-ticking nodes like "Sequence", "Fallback", or "SleepMs".
 */
class FilteredCoutLogger : public BT::StatusChangeLogger
{
public:
    FilteredCoutLogger(const BT::Tree& tree, std::vector<std::string> nodes_to_ignore = {})
      : BT::StatusChangeLogger(tree.rootNode()),
        nodes_to_ignore_(std::move(nodes_to_ignore))
    {
    }

    void callback(BT::Duration, const BT::TreeNode& node, BT::NodeStatus, BT::NodeStatus status) override
    {
        // Check if the node's registration name is in the ignore list
        const auto& node_name = node.registrationName();
        if (std::find(nodes_to_ignore_.begin(), nodes_to_ignore_.end(), node_name) != nodes_to_ignore_.end())
        {
            return; // Don't log this node
        }

        constexpr const char* GREEN = "\033[32m";
        constexpr const char* RED = "\033[31m";
        constexpr const char* ORANGE = "\033[33m";
        constexpr const char* RESET = "\033[0m";

        const char* color = RESET;
        if (status == BT::NodeStatus::SUCCESS)
        {
            color = GREEN;
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            color = RED;
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            color = ORANGE;
        }
        
        // We need to use the instance name, not the registration name, to match the Groot2 editor view.
        std::cout << color
                  << " [ " << node.name() << " ]: " << BT::toStr(status, true)
                  << RESET << std::endl;
    }

    void flush() override {}

private:
    std::vector<std::string> nodes_to_ignore_;
};

} // end namespace GrabBox 