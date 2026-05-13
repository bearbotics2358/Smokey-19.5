#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>

class FieldConstants {
public:
    inline static const frc::Translation2d kBlueHubCenter{182.11_in, 158.84_in};
    inline static const frc::Translation2d kRedHubCenter{469.11_in, 158.84_in};

    static frc::Translation2d GetHubCenterForMyAlliance() {
        frc::DriverStation::Alliance alliance =
            frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);

        if (frc::DriverStation::Alliance::kBlue == alliance) {
            return kBlueHubCenter;
        } else {
            return kRedHubCenter;
        }
    }
};