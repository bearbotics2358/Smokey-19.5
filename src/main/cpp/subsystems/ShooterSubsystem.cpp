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

ShooterSubsystem::ShooterSubsystem(){
    ConfigureShooterMotors();
    ConfigureFeederMotors();
}

void ShooterSubsystem::ConfigureShooterMotors()
{
    configs::TalonFXConfiguration configs{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 100_A;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;

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

void ShooterSubsystem::ConfigureFeederMotor()
{
    configs::TalonFXConfiguration feeder_configs{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 70_A;
    feeder_configs.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    feeder_configs.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;

    feeder_configs.CurrentLimits.StatorCurrentLimit = 70_A;
    feeder_configs.CurrentLimits.StatorCurrentLimitEnable = true;

    feeder_configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

    feeder_configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    feeder_configs.Slot0.kP = 1.0;
    feeder_configs.Slot0.kI = 0.0;
    feeder_configs.Slot0.kD = 0.0;
    feeder_configs.Slot0.kV = 0.12;

    m_FeederAMotor.GetConfigurator().Apply(feeder_configs);

    configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    m_FeederBMotor.GetConfigurator().Apply(feeder_configs);
}

void ShooterSubsystem::Periodic() {}
void ShooterSubsystem::SimulationPeriodic() {}

units::revolutions_per_minute_t GetDrumSpeed(){return 5_rpm;}
units::revolutions_per_minute_t CalculateFeederSpeed(){return 5_rpm;}//@todo:write the logic for calculating the feeder motors' speed

frc2::CommandPtr EnableShooter(){}
frc2::CommandPtr DisableShooter(){}
frc2::CommandPtr EnableDrum(){}
frc2::CommandPtr SlowDrum(){}