#include "Config.h"
#include <fstream>
#include <iostream>
#include <string>

/**
 * The RIOs on each bot should have a file under /home/lvuser/RobotType containing either
 * competition or practice. We can use that to determine various things like the CAN addresses
 * and offsets for the components specific to each robot.
 */
RobotType config::GetRobotType() {
    std::ifstream input_file("/home/lvuser/RobotType");
    std::string type_string;
    RobotType robot_type = RobotType::Competition;

    if (input_file.is_open()) {
      input_file >> type_string;
      input_file.close();
    }

    if (type_string.find("competition") != std::string::npos
        || type_string.find("Competition") != std::string::npos) {
      robot_type = RobotType::Competition;
    } else if (type_string.find("practice") != std::string::npos
               || type_string.find("Practice") != std::string::npos) {
      robot_type = RobotType::Practice;
    }

    // If the file cannot be found, select the competition bot
    return robot_type;
}