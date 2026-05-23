#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain(RobotType robotType)
{
    if (RobotType::Practice == robotType) {
        return {
            practice_bot::TunerConstants::DrivetrainConstants,
            practice_bot::TunerConstants::FrontLeft,
            practice_bot::TunerConstants::FrontRight,
            practice_bot::TunerConstants::BackLeft,
            practice_bot::TunerConstants::BackRight
        };
    } else {
        return {
            comp_bot::TunerConstants::DrivetrainConstants,
            comp_bot::TunerConstants::FrontLeft,
            comp_bot::TunerConstants::FrontRight,
            comp_bot::TunerConstants::BackLeft,
            comp_bot::TunerConstants::BackRight
        };
    }
}

units::meters_per_second_t TunerConstants::GetSpeedAt12Volts(RobotType robotType)
{
    switch (robotType) {
        case RobotType::Practice:
            return practice_bot::TunerConstants::kSpeedAt12Volts;

        case RobotType::Competition:
        default:
            return comp_bot::TunerConstants::kSpeedAt12Volts;
    }
}
