#include <iostream>
#include "ruiwo_actuator.h"

int main() {
    std::cout << "RuiWoActuator Motor Position Read Example" << std::endl;
    std::cout << "==========================================" << std::endl;

    // Create RuiWoActuator instance
    RuiWoActuator actuator;

    // Initialize the actuator
    std::cout << "Initializing actuator..." << std::endl;
    int init_result = actuator.initialize();
    if (init_result != 0) {
        std::cerr << "Failed to initialize actuator, error code: " << init_result << std::endl;
        return -1;
    }
    std::cout << "Actuator initialized successfully." << std::endl;

    // Get current motor positions
    std::vector<double> current_positions = actuator.get_positions();
    std::cout << "Current motor positions (" << current_positions.size() << " motors):" << std::endl;
    for (size_t i = 0; i < current_positions.size(); ++i) {
        std::cout << "  Motor[" << i << "]: " << current_positions[i] << " rad" << std::endl;
    }

    /////////////////////////////////////////////////////
    //  可以添加电机控制在这里
    ////////////////////////////////////////////////////

    // Close actuator
    actuator.close();
    std::cout << "Actuator closed." << std::endl;

    return 0;
}