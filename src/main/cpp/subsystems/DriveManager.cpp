#include "subsystems/DriveManager.h"
#include "bearlog/bearlog.h"
#include "subsystems/RobotZoneHelper.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <cmath>
#include <frc/Timer.h>

DriveManager::DriveManager(std::function<frc::Pose2d()> getBotPose,
                           std::function<units::radians_per_second_t()> getAngularVelocity) :
    m_GetCurrentBotPose(getBotPose),
    m_GetCurrentAngularVelocity(getAngularVelocity)
{
    m_SetpointDistance = kLeftSetpointDistance;
    m_rotationalPID.EnableContinuousInput(-180, 180);
}

bool DriveManager::AssistManagerA() {
    BearLog::Log("assist", std::string("assist running"));
    if (GoThroughTrench() == false) {
        if (AngleBump() == false) {
            BearLog::Log("Driver Assist", std::string("No Avaiable Tool"));
            return false;
        } else {
            BearLog::Log("Driver Assist", std::string("Turn For Bump"));
            return true;
        }
    } else {
        BearLog::Log("Driver Assist", std::string("Go Through Trench"));
        return true;
    };
}

bool DriveManager::GoThroughTrench() {
    RobotZoneHelper::TrenchZone inTrenchZone = RobotZoneHelper::isRobotInTrenchZone(m_GetCurrentBotPose());

    if (!(inTrenchZone == RobotZoneHelper::TrenchZone::NoTrenchZone)) {

        units::degree_t m_RotationSetpoint;
        if (inTrenchZone == RobotZoneHelper::TrenchZone::InRightTrenchZone) {
            m_SetpointDistance = kRightSetpointDistance;
            m_RotationSetpoint = 90_deg;
        } else {
            m_SetpointDistance = kLeftSetpointDistance;
            m_RotationSetpoint = -90_deg;
        }

        BearLog::Log("DriverAssist/RotationSetpoint", m_RotationSetpoint);


        frc::Pose2d botPose = m_GetCurrentBotPose();
        double robotY = m_YAlignmentPID.Calculate(botPose.Y().value(), m_SetpointDistance.value());
        robotY = std::clamp(robotY, -1.0, 1.0);

        BearLog::Log("Strafe PID", robotY);

        units::degree_t currentDegrees = botPose.Rotation().Degrees();
        double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (m_RotationSetpoint).value());
        rotation = std::clamp(rotation, -1.0, 1.0);

        BearLog::Log("Rotation PID", rotation);


        xMovement = -m_driverController.GetLeftY();
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
            yMovement = -robotY;
        } else {
            yMovement = robotY;
        }
        rotMovement = rotation;

        return true;
    } else {
        return false;
    }
}

bool DriveManager::AngleBump() {
    RobotZoneHelper::BumpZone inBumpZone = RobotZoneHelper::isRobotInBumpZone(m_GetCurrentBotPose());

    if (!(inBumpZone == RobotZoneHelper::BumpZone::NoBumpZone)) {

        units::degree_t m_RotationSetpoint;

        if (inBumpZone == RobotZoneHelper::BumpZone::InRightBumpZone) {
            m_SetpointDistance = kRightSetpointDistance;
            m_RotationSetpoint = 90_deg;
        } else {
            m_SetpointDistance = kLeftSetpointDistance;
            m_RotationSetpoint = -90_deg;
        }

        BearLog::Log("DriverAssist/RotationSetpoint", m_RotationSetpoint);

        frc::Pose2d botPose = m_GetCurrentBotPose();

        units::degree_t currentDegrees = botPose.Rotation().Degrees();

        double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (m_RotationSetpoint).value());
        rotation = std::clamp(rotation, -1.0, 1.0);

        BearLog::Log("Rotation PID", rotation);


        xMovement = -m_driverController.GetLeftY();
        yMovement = -m_driverController.GetLeftX();
        rotMovement = rotation;

        return true;
    } else {
        return false;
    }
}

bool DriveManager::TurnToHub() {
    if (RobotZoneHelper::isRobotInMyAllianceZone(m_GetCurrentBotPose())) {
        BearLog::Log("PointAtHub", true);


        frc::Pose2d botPose = m_GetCurrentBotPose();
        frc::DriverStation::Alliance currentAlliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);

        frc::Pose2d HubPose;

        if (currentAlliance == frc::DriverStation::Alliance::kRed) {
            HubPose = redHubPose;
        } else {
            HubPose = blueHubPose;
        }

        units::meter_t strafe = botPose.Y() - HubPose.Y();
        units::meter_t forward = botPose.X() - HubPose.X();
        units::degree_t angleToHub;

        double rotation;
        units::degree_t currentDegrees = botPose.Rotation().Degrees();
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
            angleToHub = units::degree_t(units::radian_t(atan2(strafe.value(), forward.value())));
            rotation = m_rotationalPID.Calculate(currentDegrees.value() + 180, (angleToHub).value());
        } else {
            angleToHub = units::degree_t(units::radian_t(atan2(strafe.value(), forward.value())));
            rotation = m_rotationalPID.Calculate(currentDegrees.value() + 180, (angleToHub).value());
        }
        rotation = std::clamp(rotation, -1.0, 1.0);

        BearLog::Log("Debugging/Rotation PID", rotation);
        BearLog::Log("Debugging/RobotAngle", botPose.Rotation().Degrees());
        BearLog::Log("Debugging/AngleToHub", angleToHub);


        xMovement = -m_driverController.GetLeftY();
        yMovement = -m_driverController.GetLeftX();
        rotMovement = rotation;

        return true;
    } else {
        BearLog::Log("PointAtHub", false);

        return false;
    }
}

bool DriveManager::SequenceTurnToHub() {

    frc::Pose2d botPose = m_GetCurrentBotPose();
    units::radians_per_second_t currentAngularVelocity = m_GetCurrentAngularVelocity();
    units::second_t currentTimestamp = frc::Timer::GetFPGATimestamp();
    if (m_HasPreviousAngularVelocity) {
        units::second_t elapsed = currentTimestamp - m_PreviousAngularVelocityTimestamp;
        if (elapsed >= 5_ms && elapsed <= 250_ms) {
            units::radians_per_second_squared_t angularAcceleration =
                (currentAngularVelocity - m_PreviousAngularVelocity) / elapsed;
            if (currentAngularVelocity.value() * angularAcceleration.value() < 0.0) {
                double observedDeceleration = std::clamp(std::abs(angularAcceleration.value()), 0.25, 20.0);
                m_breakingDeceleration = units::radians_per_second_squared_t(
                    0.8 * m_breakingDeceleration.value() + 0.2 * observedDeceleration);
            }
        }
    }
    m_PreviousAngularVelocity = currentAngularVelocity;
    m_PreviousAngularVelocityTimestamp = currentTimestamp;
    m_HasPreviousAngularVelocity = true;
    frc::DriverStation::Alliance currentAlliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);

    frc::Pose2d HubPose;

    if (currentAlliance == frc::DriverStation::Alliance::kRed) {
        HubPose = redHubPose;
    } else {
        HubPose = blueHubPose;
    }

    units::meter_t strafe = botPose.Y() - HubPose.Y();
    units::meter_t forward = botPose.X() - HubPose.X();
    units::degree_t angleToHub;

    double rotation;
    units::degree_t currentDegrees = botPose.Rotation().Degrees() + 180_deg;
    while (currentDegrees.value() > 180) {
        currentDegrees -= 360_deg;
    }
    while (currentDegrees.value() < -180) {
        currentDegrees += 360_deg;
    }
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        angleToHub = units::degree_t(units::radian_t(atan2(strafe.value(), forward.value())));
        while (angleToHub.value() > 180) {
            angleToHub -= 360_deg;
        }
        while (angleToHub.value() < -180) {
            angleToHub += 360_deg;
        }
        rotation = m_rotationalPID.Calculate(currentDegrees.value(), (angleToHub).value());
    } else {
        angleToHub = units::degree_t(units::radian_t(atan2(strafe.value(), forward.value())));
        while (angleToHub.value() > 180) {
            angleToHub -= 360_deg;
        }
        while (angleToHub.value() < -180) {
            angleToHub += 360_deg;
        }
        rotation = m_rotationalPID.Calculate(currentDegrees.value(), (angleToHub).value());
    }
    rotation = std::clamp(rotation, -1.0, 1.0);

    BearLog::Log("Debugging/Rotation PID", rotation);
    BearLog::Log("Debugging/RobotAngle", currentDegrees);
    BearLog::Log("Debugging/AngleToHub", angleToHub);


    xMovement = -m_driverController.GetLeftY();
    yMovement = -m_driverController.GetLeftX();
    rotMovement = rotation;

    units::radian_t brakingTolerance =
        units::radian_t(currentAngularVelocity.value() * kResponseDelay.value()) +
        units::radian_t(currentAngularVelocity.value() * std::abs(currentAngularVelocity.value()) /
                        (2.0 * std::abs(m_breakingDeceleration.value())));
    units::degree_t angularTolerance = angleToHub - units::degree_t(brakingTolerance);
    while (angularTolerance.value() > 180) {
        angularTolerance -= 360_deg;
    }
    while (angularTolerance.value() < -180) {
        angularTolerance += 360_deg;
    }
    double stoppingAngleError = currentDegrees.value() - angularTolerance.value();
    while (stoppingAngleError > 180) {
        stoppingAngleError -= 360;
    }
    while (stoppingAngleError < -180) {
        stoppingAngleError += 360;
    }
    if (std::abs(stoppingAngleError) < kRotationTolerance.value()) {
        return true;
    } else {
        return false;
    }
}

bool DriveManager::SequenceTurnToAlliance() {

    frc::Pose2d botPose = m_GetCurrentBotPose();
    frc::DriverStation::Alliance currentAlliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);

    frc::Pose2d HubPose;

    if (currentAlliance == frc::DriverStation::Alliance::kRed) {
        HubPose = redHubPose;
    } else {
        HubPose = blueHubPose;
    }

    units::meter_t strafe = botPose.Y() - HubPose.Y();
    units::meter_t forward = botPose.X() - HubPose.X();
    units::degree_t angleToAlliance;

    double rotation;
    units::degree_t currentDegrees = botPose.Rotation().Degrees() + 180_deg;
    while (currentDegrees.value() > 180) {
        currentDegrees -= 360_deg;
    }
    while (currentDegrees.value() < -180) {
        currentDegrees += 360_deg;
    }
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        angleToAlliance = 180_deg;
        rotation = m_rotationalPID.Calculate(currentDegrees.value(), (angleToAlliance).value());
    } else {
        angleToAlliance = 0_deg;
        rotation = m_rotationalPID.Calculate(currentDegrees.value(), (angleToAlliance).value());
    }
    rotation = std::clamp(rotation, -1.0, 1.0);

    BearLog::Log("Debugging/Rotation PID", rotation);
    BearLog::Log("Debugging/RobotAngle", currentDegrees);
    BearLog::Log("Debugging/AngleToHub", angleToAlliance);


    xMovement = -m_driverController.GetLeftY();
    yMovement = -m_driverController.GetLeftX();
    rotMovement = rotation;

    if ((abs(currentDegrees.value() - angleToAlliance.value()) < kRotationTolerance.value()) || (360 + abs(currentDegrees.value() - angleToAlliance.value()) < kRotationTolerance.value())) {
        return true;
    } else {
        return false;
    }
}

frc2::CommandPtr DriveManager::DriveAlongWall() {
    frc::Pose2d target1{0.8_m, 1_m, frc::Rotation2d(180_deg)};
    frc::Pose2d target2{0.5_m, 0.5_m, frc::Rotation2d(90_deg)};
    frc::Pose2d target3{0.5_m, 2.3_m, frc::Rotation2d(90_deg)};

    frc::Translation2d fieldCenter{325.61_in, 158.84_in};

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        target1 = target1.RotateAround(fieldCenter, frc::Rotation2d(180_deg));
        target2 = target2.RotateAround(fieldCenter, frc::Rotation2d(180_deg));
        target3 = target3.RotateAround(fieldCenter, frc::Rotation2d(180_deg));
    }

    auto slowConstraints = pathplanner::PathConstraints(
        1.0_mps, 3.0_mps_sq,
        540_deg_per_s, 720_deg_per_s_sq
    );

    auto normalConstraints = pathplanner::PathConstraints(
        3.0_mps, 3.0_mps_sq,
        540_deg_per_s, 720_deg_per_s_sq
    );

    return frc2::cmd::Sequence(
        pathplanner::AutoBuilder::pathfindToPose(target1, normalConstraints, 0_mps),
        pathplanner::AutoBuilder::pathfindToPose(target2, slowConstraints, 0_mps),
        pathplanner::AutoBuilder::pathfindToPose(target3, slowConstraints, 0_mps)
    );
}
