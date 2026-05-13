// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "LaunchHelper.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <frc2/command/RunCommand.h>

#include "bearlog/bearlog.h"
#include "subsystems/RobotZoneHelper.h"

RobotContainer::RobotContainer()
    : m_turretSubsystem{[this] { return m_drivetrain.GetState().Pose; }},
      m_driveManager{[this] { return m_drivetrain.GetState().Pose; }}
{
    // The LaunchHelper needs to be initialized when the robot code is booting up before any other calls to
    // LaunchHelper are made
    LaunchHelper::GetInstance().Init(
        // Shooter/hood angle supplier
        [this] { return m_shooterSubsystem.GetCurrentHoodAngle(); },

        // Shooter speed supplier
        [this] { return m_shooterSubsystem.GetCurrentShooterSpeed(); },

        // Turret angle supplier
        [this] { return m_turretSubsystem.CurrentAngle(); },

        // Robot speed supplier
        [this] { return m_drivetrain.GetState().Speeds; },

        // Robot pose supplier
        [this] { return m_drivetrain.GetState().Pose; }
    );

    ConfigurePathPlanner();

    m_drivetrain.ConfigureAutoBuilder();

    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

    m_pdh = std::make_shared<frc::PowerDistribution>(1, frc::PowerDistribution::ModuleType::kRev);
    BearLog::SetPdh(m_pdh);

    BearLog::SetOptions({BearLogOptions::NTPublish::Yes, BearLogOptions::LogWithNTPrefix::Yes, BearLogOptions::LogExtras::Yes});

    ConfigureBindings();

    configs::Pigeon2Configuration pigeon_config{};
    pigeon_config.GyroTrim.GyroScalarZ = 0.227;
    m_drivetrain.GetPigeon2().GetConfigurator().Apply(pigeon_config);
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.SetDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.ApplyRequest([this]() -> auto&& {
                return drive.WithVelocityX(-driverJoystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .WithVelocityY(-driverJoystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .WithRotationalRate(-driverJoystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            })
        );

    driverJoystick.A().WhileTrue(
        frc2::cmd::Run([this] {
            if (m_driveManager.AssistManagerA() == true) {
                m_drivetrain.SetControl(
                    drive.WithVelocityX(m_driveManager.xMovement * MaxSpeed) // Drive forward with negative Y (forward)
                        .WithVelocityY(m_driveManager.yMovement * MaxSpeed) // Drive left with negative X (left)
                        .WithRotationalRate(m_driveManager.rotMovement * MaxAngularRate) // Drive counterclockwise with negative X (left)
                );
            }
        }));

    driverJoystick.LeftBumper().WhileTrue(
        frc2::cmd::Run([this] {
            if (m_driveManager.TurnToHub() == true) {
                m_drivetrain.SetControl(
                    drive.WithVelocityX(m_driveManager.xMovement * MaxSpeed) // Drive forward with negative Y (forward)
                        .WithVelocityY(m_driveManager.yMovement * MaxSpeed) // Drive left with negative X (left)
                        .WithRotationalRate(m_driveManager.rotMovement * MaxAngularRate) // Drive counterclockwise with negative X (left)
                );
            }
        }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        m_drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    driverJoystick.X().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    driverJoystick.Y().OnTrue(m_intakeSubsystem.RunIntakeInReverse());
    driverJoystick.Y().OnFalse(m_intakeSubsystem.StopIntake());

    driverJoystick.POVUp().OnTrue(m_indexerSubsystem.RunIndexerInReverse());
    driverJoystick.POVUp().OnFalse(m_indexerSubsystem.Stop());

    operatorJoystick.X().OnTrue(m_hopperSubsystem.ExtendHopper());
    operatorJoystick.X().OnFalse(m_hopperSubsystem.StopHopper());
    operatorJoystick.B().OnTrue(m_hopperSubsystem.StowHopper());
    operatorJoystick.B().OnFalse(m_hopperSubsystem.StopHopper());

    operatorJoystick.POVLeft().OnTrue(m_turretSubsystem.NudgeOffsetUp());
    operatorJoystick.POVRight().OnTrue(m_turretSubsystem.NudgeOffsetDown());

    operatorJoystick.RightTrigger().WhileTrue(m_hopperSubsystem.AgitateToHelpIndexer());
    operatorJoystick.RightTrigger().OnFalse(m_hopperSubsystem.StopHopper());

    operatorJoystick.LeftTrigger().OnFalse(m_intakeSubsystem.StopIntake());
    operatorJoystick.LeftTrigger().OnTrue(m_intakeSubsystem.RunIntake());

    operatorJoystick.LeftBumper().OnTrue(m_intakeSubsystem.RunIntakeInReverse());
    operatorJoystick.LeftBumper().OnFalse(m_intakeSubsystem.StopIntake());

    driverJoystick.LeftTrigger().OnFalse(m_intakeSubsystem.StopIntake());
    driverJoystick.LeftTrigger().OnTrue(m_intakeSubsystem.RunIntake());

    driverJoystick.RightBumper().OnTrue(
        frc2::cmd::Sequence(
            frc2::cmd::Either(
                m_shooterSubsystem.EnableShooterWithFixedHoodAngle().WithTimeout(0.05_s),
                m_shooterSubsystem.EnableShooterWithFixedHoodAndFixedSpeed().WithTimeout(0.05_s),
                [this] {return RobotZoneHelper::isRobotInMyAllianceZone(m_drivetrain.GetState().Pose);}
            ),
            m_indexerSubsystem.RunIndexerForLaunching().WithTimeout(0.05_s)
        ).AndThen(
            frc2::cmd::Parallel(
                frc2::cmd::Either(
                    m_shooterSubsystem.EnableShooterWithFixedHoodAngle(),
                    m_shooterSubsystem.EnableShooterWithFixedHoodAndFixedSpeed(),
                    [this] {return RobotZoneHelper::isRobotInMyAllianceZone(m_drivetrain.GetState().Pose);}
                ),
                m_indexerSubsystem.RunIndexerForLaunching())
        )).OnFalse(
        frc2::cmd::Parallel(
            m_shooterSubsystem.StopShooter(),
            m_indexerSubsystem.Stop()
        )
    );

    driverJoystick.RightTrigger().WhileTrue(
        frc2::cmd::Sequence(
            m_shooterSubsystem.EnableShooterWithFixedHoodAndFixedSpeed().WithTimeout(0.05_s),
            m_indexerSubsystem.RunIndexerForLaunching().WithTimeout(0.05_s)
        ).AndThen(
            frc2::cmd::Parallel(
                m_shooterSubsystem.EnableShooterWithFixedHoodAndFixedSpeed(),
                m_indexerSubsystem.RunIndexerForLaunching())
        )).OnFalse(
        frc2::cmd::Parallel(
            m_shooterSubsystem.StopShooter(),
            m_indexerSubsystem.Stop()
        )
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (driverJoystick.Back() && driverJoystick.Y()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (driverJoystick.Back() && driverJoystick.X()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (driverJoystick.Start() && driverJoystick.Y()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (driverJoystick.Start() && driverJoystick.X()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading
    driverJoystick.POVDown().OnTrue(m_drivetrain.RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    operatorJoystick.Y().OnTrue(m_turretSubsystem.ZeroTurret());
    operatorJoystick.POVUp().WhileTrue(m_FMSSubsystem.ManualShift("Red"));
    operatorJoystick.POVDown().WhileTrue(m_FMSSubsystem.ManualShift("Blue"));
    operatorJoystick.A().OnTrue(m_turretSubsystem.PointAtHub());

    //Don't use until tested
    //operatorJoystick.B().OnTrue(m_shooterSubsystem.CalibrateHoodMotor());
    driverJoystick.POVLeft().WhileTrue(m_driveManager.DriveAlongWall());
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}

void RobotContainer::ConfigurePathPlanner() {
    const units::second_t kLaunchTime = 12_s;
    using namespace pathplanner;
    NamedCommands::registerCommand(
        "Extend Hopper",
        std::move(m_hopperSubsystem.ExtendHopper().WithTimeout(1_s))
    );
    NamedCommands::registerCommand(
        "Squeeze Hopper",
        std::move(m_hopperSubsystem.AgitateToHelpIndexer().WithTimeout(kLaunchTime))
    );
    NamedCommands::registerCommand(
        "Run Intake",
        std::move(m_intakeSubsystem.RunIntake())
    );
    NamedCommands::registerCommand(
        "Stop Intake",
        std::move(m_intakeSubsystem.StopIntake())
    );
    NamedCommands::registerCommand(
        "Stop Shooter",
        std::move(m_shooterSubsystem.StopShooter())
    );
    NamedCommands::registerCommand(
        "Point Turret at Hub",
        std::move(m_turretSubsystem.PointAtHub())
    );
    NamedCommands::registerCommand(
        "Launch Fuel at Hub",
        std::move(
            frc2::cmd::Parallel(
                m_shooterSubsystem.EnableShooterWithFixedHoodAngle(),
                m_indexerSubsystem.RunIndexerForLaunching()
            ).WithTimeout(kLaunchTime)
        )
    );
    NamedCommands::registerCommand(
        "Run Indexer",
        std::move(
            m_indexerSubsystem.RunIndexerForLaunching()
        )
    );
    NamedCommands::registerCommand(
        "Stop Indexer",
        std::move(
            m_indexerSubsystem.Stop()
        )
    );
    NamedCommands::registerCommand(
        "Empty Hopper",
        std::move(
            m_intakeSubsystem.RunIntakeInReverse()
        )
    );

    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

    pathplanner::AutoBuilder::configure(
        [this]() { return m_drivetrain.GetState().Pose; },
        [this](frc::Pose2d pose) { m_drivetrain.ResetPose(pose); },
        [this]() { return m_drivetrain.GetState().Speeds; },
        [this](frc::ChassisSpeeds speeds) {
            m_drivetrain.SetControl(
                drive.WithVelocityX(speeds.vx)
                    .WithVelocityY(speeds.vy)
                    .WithRotationalRate(speeds.omega)
            );
        },

        std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants(5.0, 0.0, 0.0),
            pathplanner::PIDConstants(5.0, 0.0, 0.0)
        ),

        config,
        []() { return true; },
        &m_drivetrain
    );
}
