#include "subsystems/DriveManager.h"
#include "bearlog/bearlog.h"
#include "subsystems/RobotZoneHelper.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

DriveManager::DriveManager(std::function<frc::Pose2d()> getBotPose) :
    m_GetCurrentBotPose(getBotPose)
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
        if (inTrenchZone == RobotZoneHelper::TrenchZone::InRightTrenchZone) {
            m_SetpointDistance = kRightSetpointDistance;
        } else {
            m_SetpointDistance = kLeftSetpointDistance;
        }


        frc::Pose2d botPose = m_GetCurrentBotPose();
        double robotY = m_YAlignmentPID.Calculate(botPose.Y().value(), m_SetpointDistance.value());
        robotY = std::clamp(robotY, -1.0, 1.0);

        BearLog::Log("Strafe PID", robotY);

        units::degree_t currentDegrees = botPose.Rotation().Degrees();
        double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (0_deg).value());
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
        frc::Pose2d botPose = m_GetCurrentBotPose();

        units::degree_t currentDegrees = botPose.Rotation().Degrees();

        double rotation = m_rotationalPID.Calculate(currentDegrees.value(), (45_deg).value());
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
        units::degree_t offset = 0_deg;

        if (currentAlliance == frc::DriverStation::Alliance::kRed) {
            HubPose = redHubPose;
            offset = 180_deg;
        } else {
            HubPose = blueHubPose;
        }

        units::meter_t strafe = botPose.Y() - HubPose.Y();
        units::meter_t forward = botPose.X() - HubPose.X();
        units::degree_t angleToHub;

        double rotation;
        units::degree_t currentDegrees = botPose.Rotation().Degrees();
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
            angleToHub = units::degree_t(units::radian_t(atan2(strafe.value(), forward.value()))) + offset;
            rotation = m_rotationalPID.Calculate(currentDegrees.value() + 180, (angleToHub).value());
        } else {
            angleToHub = units::degree_t(units::radian_t(atan2(strafe.value(), forward.value()))) + offset;
            rotation = m_rotationalPID.Calculate(currentDegrees.value(), (angleToHub).value());
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