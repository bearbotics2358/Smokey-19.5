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
    : m_driveManager{[this] { return m_drivetrain.GetState().Pose; }}
{
    // The LaunchHelper needs to be initialized when the robot code is booting up before any other calls to
    // LaunchHelper are made
    LaunchHelper::GetInstance().Init(
    // Robot speed supplier
        [this] { return m_drivetrain.GetState().Speeds; },

        // Robot pose supplier
        [this] { return m_drivetrain.GetState().Pose; }
    );

    ConfigurePathPlanner();

    m_drivetrain.ConfigureAutoBuilder();

    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);

    // @todo Re-enable PDH logging after figuring out why it is broken
    // m_pdh = std::make_shared<frc::PowerDistribution>(1, frc::PowerDistribution::ModuleType::kRev);
    // BearLog::SetPdh(m_pdh);

    BearLog::SetOptions({BearLogOptions::NTPublish::Yes, BearLogOptions::LogWithNTPrefix::Yes, BearLogOptions::LogExtras::No});

    ConfigureBindings();
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


    //TODO: Change the keybind to something that makes sense
    driverJoystick.POVUp().WhileTrue(
        m_conveyorPivotSubsystem.Extend()
    ).OnFalse(
        m_conveyorPivotSubsystem.Stop()
    );
    driverJoystick.B().WhileTrue(
        RetractPivotCommand()
    ).OnFalse(
        StopPivotCommand()
    );

    driverJoystick.RightBumper().WhileTrue(
        frc2::cmd::Parallel(
            m_shooterSubsystem.RunDrumAndFeeder(),
            m_conveyorBeltSubsystem.RunBelt()
        )
    ).OnFalse(
        frc2::cmd::Parallel(
            m_shooterSubsystem.RunDrumSlowly(),
            m_conveyorBeltSubsystem.Stop()
        )
    );

    driverJoystick.RightTrigger().WhileTrue(
        frc2::cmd::Run(
            [this] {
                m_driveManager.TurnToHub();
                m_drivetrain.SetControl(
                    drive.WithVelocityX(m_driveManager.xMovement * MaxSpeed)
                        .WithVelocityY(m_driveManager.yMovement * MaxSpeed)
                        .WithRotationalRate(m_driveManager.rotMovement * MaxAngularRate)
                );
            }
        )
        .WithTimeout(1_s)
        .AndThen(
            m_shooterSubsystem.RunDrumAndFeeder()
                .AlongWith(
                    m_conveyorBeltSubsystem.RunBelt()
                )
                .AlongWith(
                    m_drivetrain.ApplyRequest(
                        [this]() -> auto&& {
                            return brake;
                        }
                    )
                )
        )
    ).OnFalse(
        frc2::cmd::Parallel(
            m_shooterSubsystem.RunDrumSlowly(),
            m_conveyorBeltSubsystem.Stop()
        )
    );

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

    driverJoystick.LeftTrigger().OnTrue(
        frc2::cmd::Parallel(
            m_intakeSubsystem.RunIntake(),
            m_conveyorBeltSubsystem.RunBelt()
        )
    ).OnFalse(
        frc2::cmd::Parallel(
            m_intakeSubsystem.StopIntake(),
            m_conveyorBeltSubsystem.Stop()
        )
    );

    operatorJoystick.X().OnTrue(
        m_conveyorPivotSubsystem.Extend()
    ).OnFalse(
        m_conveyorPivotSubsystem.Stop()
    );

    operatorJoystick.B().OnTrue(
        RetractPivotCommand()
    ).OnFalse(
        StopPivotCommand()
    );

    operatorJoystick.LeftTrigger().OnTrue(
        m_intakeSubsystem.RunIntake()
    ).OnFalse(
        m_intakeSubsystem.StopIntake()
    );

    operatorJoystick.LeftBumper().OnTrue(
        m_intakeSubsystem.RunIntakeInReverse()
    ).OnFalse(
        m_intakeSubsystem.StopIntake()
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

    operatorJoystick.POVUp().WhileTrue(m_FMSSubsystem.ManualShift("Red"));
    operatorJoystick.POVDown().WhileTrue(m_FMSSubsystem.ManualShift("Blue"));

    driverJoystick.POVLeft().WhileTrue(m_driveManager.DriveAlongWall());
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    return m_autoChooser.GetSelected();
}

frc2::CommandPtr RobotContainer::RetractPivotCommand() {
    return frc2::cmd::Parallel(
        m_conveyorPivotSubsystem.Stow(),
        m_conveyorBeltSubsystem.RunBelt()
    );
}

frc2::CommandPtr RobotContainer::StopPivotCommand() {
    return frc2::cmd::Parallel(
        m_conveyorPivotSubsystem.Stop(),
        m_conveyorBeltSubsystem.Stop()
    );
}

void RobotContainer::ConfigurePathPlanner() {
    const units::second_t kLaunchTime = 12_s;
    using namespace pathplanner;
    NamedCommands::registerCommand(
        "Launch",
        std::move(m_shooterSubsystem.RunDrumAndFeeder().WithTimeout(kAutoLaunchTime))
    );
    NamedCommands::registerCommand(
        "Standby",
        std::move(m_shooterSubsystem.RunDrumSlowly())
    );
    NamedCommands::registerCommand(
        "Extend Pivot",
        std::move(m_conveyorPivotSubsystem.Extend())
    );
    NamedCommands::registerCommand(
        "Retract Pivot",
        std::move(m_conveyorPivotSubsystem.Stow())//@todo:incorporate running the conveyor belts and intake with this so that they compressed fuel are being constantly fed through
    );
    NamedCommands::registerCommand(
        "Run Intake",
        std::move(m_intakeSubsystem.RunIntake())
    );
    NamedCommands::registerCommand(
        "Stop Intake",
        std::move(m_intakeSubsystem.StopIntake())
    );
}