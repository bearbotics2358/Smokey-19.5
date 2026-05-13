// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/PowerDistribution.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include "Config.h"
#include "subsystems/CameraSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/HopperSubsystem.h"
#include "subsystems/IndexerSubsystem.h"
#include "vision/VisionConstants.h"
#include "vision/VisionSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/FMSSubsystem.h"
#include "subsystems/DriveManager.h"

class RobotContainer {
private:
    RobotType m_RobotType{config::GetRobotType()};

    frc::SendableChooser<frc2::Command *> m_autoChooser;

    double speedlimit = 1.0;
    units::meters_per_second_t MaxSpeed = speedlimit * TunerConstants::GetSpeedAt12Volts(m_RobotType); // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = speedlimit * 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController driverJoystick{0};
    frc2::CommandXboxController operatorJoystick{1};

    TurretSubsystem m_turretSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    HopperSubsystem m_hopperSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    FMSSubsystem m_FMSSubsystem;
    DriveManager m_driveManager;

    std::shared_ptr<frc::PowerDistribution> m_pdh;

    subsystems::CommandSwerveDrivetrain m_drivetrain{TunerConstants::CreateDrivetrain(m_RobotType)};
    VisionSubsystem m_VisionSubsystem{&m_drivetrain, VisionConstants::GetLocalizationCameras(&m_drivetrain)};

public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
private:
    void ConfigureBindings();
    void ConfigurePathPlanner();
};
