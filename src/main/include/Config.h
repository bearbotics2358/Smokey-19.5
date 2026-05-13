#pragma once

#include <map>
#include <string>
#include <string_view>

enum class RobotType {
  Competition,
  Practice
};

namespace config {
  RobotType GetRobotType();

  struct CANAddress {
    int can_id;
    std::string_view bus_name;

    CANAddress() = delete;
    constexpr CANAddress(int canId, const std::string_view& busName)
      : can_id(canId),
        bus_name(busName)
    {}
  };

  static inline int GetCANAddress(const CANAddress& compBot, const CANAddress& practiceBot, RobotType instance) {
    if (RobotType::Practice == instance) {
      return compBot.can_id;
    } else {
      return practiceBot.can_id;
    }
  }

  static inline std::string_view GetCANBusName(const CANAddress& compBot, const CANAddress& practiceBot, RobotType instance) {
    if (RobotType::Practice == instance) {
      return compBot.bus_name;
    } else {
      return practiceBot.bus_name;
    }
  }
}