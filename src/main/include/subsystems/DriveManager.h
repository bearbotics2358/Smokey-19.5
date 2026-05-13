// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include "subsystems/CommandSwerveDrivetrain.h"


class DriveManager : public frc2::SubsystemBase {
public:
    DriveManager(std::function<frc::Pose2d()> getBotPose);

    double xMovement = -m_driverController.GetLeftY();
    double yMovement = -m_driverController.GetLeftX();
    double rotMovement = m_driverController.GetRightX();

    bool AssistManagerA();

    bool GoThroughTrench();
    bool AngleBump();
    bool TurnToHub();

    frc2::CommandPtr DriveAlongWall();
private:
    frc::Pose2d blueHubPose = frc::Pose2d(4.625594_m, 4.034663_m, frc::Rotation2d{});
    frc::Pose2d redHubPose = frc::Pose2d(11.915394_m, 4.034663_m, frc::Rotation2d{});
    
    frc2::CommandXboxController m_driverController{0};

    std::function<frc::Pose2d()> m_GetCurrentBotPose;

    static constexpr double kP = 1.75;
    static constexpr double kI = 0.0;
    static constexpr double kD = 0.0;
    
    
    frc::PIDController m_YAlignmentPID {kP, kI, kD};

    static constexpr double kRotationP = 0.05;
    static constexpr double kRotationI = 0.0;
    static constexpr double kRotationD = 0.001;
    frc::PIDController m_rotationalPID {kRotationP, kRotationI, kRotationD};

    static constexpr units::meters_per_second_t kMaxVelocity = 1.5_mps;
    static constexpr units::radians_per_second_t kMaxAngularVelocity = 1_rad_per_s;

    const units::meter_t kStrafeTolerance = units::meter_t(0.5_in);
    const units::degree_t kRotationTolerance = 2_deg;

    const units::meter_t kLeftSetpointDistance = units::meter_t(291.47_in);
    const units::meter_t kRightSetpointDistance = units::meter_t(26.22_in);
    units::meter_t m_SetpointDistance;
};