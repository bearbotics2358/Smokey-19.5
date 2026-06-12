#include "subsystems/ShooterSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "LaunchHelper.h"
#include <frc2/command/button/RobotModeTriggers.h>

using namespace ctre::phoenix6;

// Spinning up the drum takes time and also spikes the current usage. To avoid starting and stopping the
// drum throughout the match, always keep it running at a minimum of this speed to keep current usage in check.
const units::revolutions_per_minute_t kMinimumDrumSpeed = 1500_rpm;

ShooterSubsystem::ShooterSubsystem()
{
    ConfigureDrumMotors();
    ConfigureFeederMotors();

    // Be sure to stop all the motors if the robot is disabled while it's running
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        DisableDrumAndFeeder().IgnoringDisable(true)
    );

    //Sets the default for the drum while enabled to be at the minimum speed, insteading of coming to a stop
    frc2::RobotModeTriggers::Autonomous().OnTrue(
        RunDrumSlowly()
    );

    frc2::RobotModeTriggers::Teleop().OnTrue(
        RunDrumSlowly()
    );
}

void ShooterSubsystem::ConfigureDrumMotors()
{
    configs::TalonFXConfiguration configs{};

    // @todo Find out if this is a good stator current limit. It may need to be higher or lower depending on testing.
    configs.CurrentLimits.StatorCurrentLimit = 100_A;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    // @todo Find out if this is a good supply current limit. It may need to be higher or lower depending on testing.
    configs.CurrentLimits.SupplyCurrentLimit = 50_A;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

    configs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    configs.Slot0.kP = 0.45;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kV = 0.12;

    m_DrumAMotor.GetConfigurator().Apply(configs);
    m_DrumBMotor.GetConfigurator().Apply(configs);

    configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    m_DrumCMotor.GetConfigurator().Apply(configs);
    m_DrumDMotor.GetConfigurator().Apply(configs);
}

void ShooterSubsystem::ConfigureFeederMotors()
{
    configs::TalonFXConfiguration feeder_configs{};

    // @todo Find out if this is a good stator current limit. It may need to be higher or lower depending on testing.
    feeder_configs.CurrentLimits.StatorCurrentLimit = 80_A;
    feeder_configs.CurrentLimits.StatorCurrentLimitEnable = true;

    // @todo Find out if this is a good supply current limit. It may need to be higher or lower depending on testing.
    feeder_configs.CurrentLimits.SupplyCurrentLimit = 50_A;
    feeder_configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    feeder_configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

    feeder_configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    feeder_configs.Slot0.kP = 0.1;
    feeder_configs.Slot0.kI = 0.0;
    feeder_configs.Slot0.kD = 0.0;
    feeder_configs.Slot0.kV = 0.12;

    m_FeederAMotor.GetConfigurator().Apply(feeder_configs);

    feeder_configs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    m_FeederBMotor.GetConfigurator().Apply(feeder_configs);
    m_RollerBedMotor.GetConfigurator().Apply(feeder_configs);
}

void ShooterSubsystem::Periodic()
{
    BearLog::Log("Shooter/Drum/Speed", GetDrumSpeed());
    BearLog::Log("Shooter/Drum/SetPointSpeed", units::revolutions_per_minute_t(m_DrumVelocityVoltage.Velocity));
    BearLog::Log("Shooter/Feeder/Speed", GetFeederSpeed());
    BearLog::Log("Shooter/Feeder/SetPointSpeed", units::revolutions_per_minute_t(m_FeederVelocityVoltage.Velocity));
    LaunchHelper::GetInstance().GetLaunchParameters();
}

void ShooterSubsystem::SimulationPeriodic()
{
}

units::revolutions_per_minute_t ShooterSubsystem::GetDrumSpeed()
{
    return m_DrumAMotor.GetVelocity().GetValue();
}

units::revolutions_per_minute_t ShooterSubsystem::GetFeederSpeed()
{
    return m_FeederAMotor.GetVelocity().GetValue();
}

void ShooterSubsystem::SetGoalSpeeds(units::revolutions_per_minute_t drumSpeed, EnableFeeder enableFeeder)
{
    units::revolutions_per_minute_t drum_speed_to_set = drumSpeed; //units::math::max(drumSpeed, kMinimumDrumSpeed);

    controls::VelocityVoltage drum_velocity_request = m_DrumVelocityVoltage.WithVelocity(drum_speed_to_set);
    m_DrumAMotor.SetControl(drum_velocity_request);
    m_DrumBMotor.SetControl(drum_velocity_request);
    m_DrumCMotor.SetControl(drum_velocity_request);
    m_DrumDMotor.SetControl(drum_velocity_request);

    if (EnableFeeder::Yes == enableFeeder) {
        // Using a constant speed for the feeder velocity since it shouldn't need to change
        units::revolutions_per_minute_t feeder_velocity = drum_speed_to_set * 1.5;
        controls::VelocityVoltage feeder_velocity_request = m_FeederVelocityVoltage.WithVelocity(feeder_velocity);
        m_FeederAMotor.SetControl(feeder_velocity_request);
        m_FeederBMotor.SetControl(feeder_velocity_request);

        units::revolutions_per_minute_t roller_bed_velocity = feeder_velocity;
        controls::VelocityVoltage roller_bed_velocity_request = m_RollerBedVelocityVoltage.WithVelocity(roller_bed_velocity);
        m_RollerBedMotor.SetControl(roller_bed_velocity_request);
    } else {
        StopFeederMotors();
    }
}

frc2::CommandPtr ShooterSubsystem::TestDrum()
{
    return Run([this] {
        const units::volt_t kTest = 2_V;
        m_DrumAMotor.SetVoltage(kTest);
        m_DrumBMotor.SetVoltage(kTest);
        m_DrumCMotor.SetVoltage(kTest);
        m_DrumDMotor.SetVoltage(kTest);
    });
}

frc2::CommandPtr ShooterSubsystem::TestFeeder()
{
    return Run([this] {
        const units::volt_t kTest = 2_V;
        m_FeederAMotor.SetVoltage(kTest);
        m_FeederBMotor.SetVoltage(kTest);
    });
}

frc2::CommandPtr ShooterSubsystem::TestRollerBed()
{
    return Run([this] {
        m_RollerBedMotor.SetVoltage(2_V);
    });
}

frc2::CommandPtr ShooterSubsystem::RunDrumAndFeeder()
{
    return Run([this] {
        TrajectoryInfo parameters = LaunchHelper::GetInstance().GetLaunchParameters();

        SetGoalSpeeds(parameters.wheel_rpm, EnableFeeder::Yes);
        //SetGoalSpeeds(kMinimumDrumSpeed, EnableFeeder::Yes);
    });
}

frc2::CommandPtr ShooterSubsystem::RunDrumSlowly()
{
    return Run([this] {
        // When we're not intending to launch fuel, keep the drum running at an idle speed to avoid
        // the current spike and extra time used when spinning it up.
        SetGoalSpeeds(kMinimumDrumSpeed, EnableFeeder::No);
    });
}

frc2::CommandPtr ShooterSubsystem::DisableDrumAndFeeder()
{
    return RunOnce([this] {
        StopDrumMotors();
        StopFeederMotors();
    });
}

void ShooterSubsystem::StopDrumMotors()
{
    m_DrumAMotor.SetControl(m_Stop);
    m_DrumBMotor.SetControl(m_Stop);
    m_DrumCMotor.SetControl(m_Stop);
    m_DrumDMotor.SetControl(m_Stop);
}

void ShooterSubsystem::StopFeederMotors()
{
    m_FeederAMotor.SetControl(m_Stop);
    m_FeederBMotor.SetControl(m_Stop);
    m_RollerBedMotor.SetControl(m_Stop);
}
