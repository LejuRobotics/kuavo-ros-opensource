#pragma once

#include <vector>

namespace joint_test_poses {

    constexpr int num_joint = 28;
    
    // Change this to a function that returns the initialized vector
    inline std::vector<std::vector<double>> test_pos_list() {
        std::vector<std::vector<double>> poses;
        
        // First pose: test leg
        std::vector<double> pos1(num_joint, 0);
        pos1[0] = -2.86;
        pos1[1] = 28.65;
        pos1[2] = -74.48;
        pos1[3] = 108.86;
        pos1[4] = -17.19;
        pos1[6] = 2.86;
        pos1[7] = -28.65;
        pos1[8] = -74.48;
        pos1[9] = 108.86;
        pos1[10] = -17.19;
        pos1[26] = 44.69;
        poses.push_back(pos1);

        // Second pose: test arm
        std::vector<double> pos2(num_joint, 0);
        pos2[12] = -57.30;
        pos2[13] = -14.32;
        pos2[15] = -45.84;
        pos2[17] = -25.78;
        pos2[18] = -40.11;
        pos2[19] = -57.30;
        pos2[20] = 14.32;
        pos2[22] = -45.84;
        pos2[24] = -25.78;
        pos2[25] = 40.11;
        pos2[27] = 20.05;
        poses.push_back(pos2);

        // Third pose: test head
        std::vector<double> pos3(num_joint, 0);
        pos3[26] = 60;
        pos3[27] = 25;
        poses.push_back(pos3);

        return poses;
    }

} // namespace joint_test_poses