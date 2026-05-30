#include "subsystems/ShooterSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "LaunchHelper.h"

using namespace ctre::phoenix6;

ShooterSubsystem::ShooterSubsystem()
{
    ConfigureDrumMotors();
    ConfigureFeederMotors();
}

void ShooterSubsystem::ConfigureDrumMotors()
{
    configs::TalonFXConfiguration configs{};

    // @todo Find out if this is a good stator current limit. It may need to be higher or lower depending on testing.
    configs.CurrentLimits.StatorCurrentLimit = 80_A;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    // @todo Find out if this is a good supply current limit. It may need to be higher or lower depending on testing.
    configs.CurrentLimits.SupplyCurrentLimit = 50_A;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

    configs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    configs.Slot0.kP = 100.0;
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

    feeder_configs.Slot0.kP = 1.0;
    feeder_configs.Slot0.kI = 0.0;
    feeder_configs.Slot0.kD = 0.0;
    feeder_configs.Slot0.kV = 0.12;

    m_FeederAMotor.GetConfigurator().Apply(feeder_configs);

    feeder_configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    m_FeederBMotor.GetConfigurator().Apply(feeder_configs);
}

void ShooterSubsystem::Periodic()
{
    BearLog::Log("Shooter/Drum/Speed", GetDrumSpeed());
    BearLog::Log("Shooter/Drum/SetPointSpeed", units::revolutions_per_minute_t(m_DrumVelocityVoltage.Velocity));
    BearLog::Log("Shooter/Feeder/Speed", GetFeederSpeed());
    BearLog::Log("Shooter/Feeder/SetPointSpeed", units::revolutions_per_minute_t(m_FeederVelocityVoltage.Velocity));
}

void ShooterSubsystem::SimulationPeriodic()
{
}

units::revolutions_per_minute_t ShooterSubsystem::GetDrumSpeed()
{
    units::revolutions_per_minute_t speed = m_DrumAMotor.GetVelocity().GetValue();
    return speed;
}

units::revolutions_per_minute_t ShooterSubsystem::GetFeederSpeed()
{
    units::revolutions_per_minute_t speed = m_FeederAMotor.GetVelocity().GetValue();
    return speed;
}

void ShooterSubsystem::SetGoalSpeeds(units::revolutions_per_minute_t drumSpeed, EnableFeeder enableFeeder)
{
    // Spinning up the drum takes time and also spikes the current usage. To avoid starting and stopping the
    // drum throughout the match, always keep it running at a minimum of this speed to keep current usage in check.
    units::revolutions_per_minute_t kMinimumDrumSpeed = 1000_rpm;
    units::revolutions_per_minute_t drum_speed_to_set = units::math::max(drumSpeed, kMinimumDrumSpeed);

    controls::VelocityVoltage drum_velocity_request = m_DrumVelocityVoltage.WithVelocity(drum_speed_to_set);
    m_DrumAMotor.SetControl(drum_velocity_request);
    m_DrumBMotor.SetControl(drum_velocity_request);
    m_DrumCMotor.SetControl(drum_velocity_request);
    m_DrumDMotor.SetControl(drum_velocity_request);

    if (EnableFeeder::Yes == enableFeeder) {
        // Using a constant speed for the feeder velocity since it shouldn't need to change
        units::revolutions_per_minute_t kFeederVelocity = 4500_rpm;
        controls::VelocityVoltage feeder_velocity_request = m_FeederVelocityVoltage.WithVelocity(kFeederVelocity);
        m_FeederAMotor.SetControl(feeder_velocity_request);
        m_FeederBMotor.SetControl(feeder_velocity_request);
    } else {
        StopFeederMotors();
    }
}

frc2::CommandPtr ShooterSubsystem::RunDrumAndFeeder()
{
    return Run([this] {
        TrajectoryInfo parameters = LaunchHelper::GetInstance().GetLaunchParameters();
        SetGoalSpeeds(parameters.wheel_rpm, EnableFeeder::Yes);
    });
}

frc2::CommandPtr ShooterSubsystem::RunDrumOnly()
{
    return Run([this] {
        TrajectoryInfo parameters = LaunchHelper::GetInstance().GetLaunchParameters();
        SetGoalSpeeds(parameters.wheel_rpm, EnableFeeder::No);
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
}
