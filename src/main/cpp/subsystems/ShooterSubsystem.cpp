#include "subsystems/ShooterSubsystem.h"
#include "LaunchHelper.h"
#include "bearlog/bearlog.h"
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>
#include <frc2/command/button/RobotModeTriggers.h>

using namespace ctre::phoenix6;
ShooterSubsystem::ShooterSubsystem()
{
    ConfigureShooterMotors();
    ConfigureHoodMotor();
    ConfigureFeederMotor();

    // m_isHardStop =  frc2::Trigger([this] {
    //     return abs(m_HoodMotor.GetVelocity().GetValue().value()) < 1 &&
    //         abs(m_HoodMotor.GetTorqueCurrent().GetValue().value()) > 10;
    // }).Debounce(0.1_s);

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }

    // frc2::RobotModeTriggers::Autonomous().OnTrue(
    //     frc2::cmd::RunOnce([this] {
    //         if (false == m_HoodZeroed) {
    //             m_HoodOffset = GetAngleFromTurns(m_HoodMotor.GetPosition().GetValue()) - 75_deg;
    //             m_HoodZeroed = true;
    //         }
    //     })
    // );

    // frc2::RobotModeTriggers::Teleop().OnTrue(
    //     frc2::cmd::RunOnce([this] {
    //         if (false == m_HoodZeroed) {
    //             m_HoodOffset = GetAngleFromTurns(m_HoodMotor.GetPosition().GetValue()) - 75_deg;
    //             m_HoodZeroed = true;
    //         }
    //     })
    // );
}

// frc2::CommandPtr ShooterSubsystem::CalibrateHoodMotor() {
//     return Run([this] {
//         m_HoodMotor.SetControl(calibrationRequest);
//     }
//     ).Until(
//         [this] { return m_isHardStop.Get(); }
//     ).AndThen(
//         StopHood()
//     ).AndThen(
//         frc2::cmd::RunOnce([this] {
//             m_HoodOffset = -GetAngleFromTurns(m_HoodMotor.GetPosition().GetValue()) + 75_deg;
//         })
//     );
// }

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

    m_FlywheelMotor.GetConfigurator().Apply(configs);

    configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    m_FlywheelFollowerMotor.GetConfigurator().Apply(configs);
}

void ShooterSubsystem::ConfigureHoodMotor()
{
    configs::TalonFXConfiguration hood_config{};

    hood_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    hood_config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    hood_config.CurrentLimits.StatorCurrentLimit = 10_A;
    hood_config.CurrentLimits.StatorCurrentLimitEnable = true;

    hood_config.Slot0.kP = 8.0;
    hood_config.Slot0.kI = 4.0;
    hood_config.Slot0.kD = 0.2;
    hood_config.Slot0.kS = 0.0;
    hood_config.Slot0.kV = 0.12;

    //m_HoodMotor.GetConfigurator().Apply(hood_config);
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

    m_FeederMotor.GetConfigurator().Apply(feeder_configs);
}

void ShooterSubsystem::Periodic() {
    BearLog::Log("Shooter/Flywheel/SetPointSpeed", units::revolutions_per_minute_t(m_ShooterVelocityVoltage.Velocity));
    BearLog::Log("Shooter/Flywheel/Speed", GetCurrentShooterSpeed());
    BearLog::Log("Shooter/Flywheel/Voltage", m_FlywheelMotor.GetMotorVoltage().GetValue());

    BearLog::Log("Shooter/Flywheel Follower/Speed", units::revolutions_per_minute_t(m_FlywheelFollowerMotor.GetVelocity().GetValue()));
    BearLog::Log("Shooter/Flywheel Follower/Voltage", m_FlywheelFollowerMotor.GetMotorVoltage().GetValue());

    BearLog::Log("Shooter/Elevation/SetpointAngle", GetAngleFromTurns(m_HoodPositionVoltage.Position));
    BearLog::Log("Shooter/Elevation/CurrentAngle", GetCurrentHoodAngle());
    //BearLog::Log("Shooter/Elevation/RawCurrentAngle", GetAngleFromTurns(m_HoodMotor.GetPosition().GetValue()));
    BearLog::Log("Shooter/Elevation/HoodOffset", m_HoodOffset);
    BearLog::Log("Shooter/Elevation/HardStop", m_isHardStop.Get());

    BearLog::Log("Shooter/Feeder/Speed", units::revolutions_per_minute_t(m_FeederMotor.GetVelocity().GetValue()));
    BearLog::Log("Shooter/Feeder/SetPointSpeed", units::revolutions_per_minute_t(m_FeederVelocityVoltage.Velocity));
    BearLog::Log("Shooter/Feeder/Current", m_FeederMotor.GetTorqueCurrent().GetValue());
}

units::degree_t ShooterSubsystem::GetCurrentHoodAngle() {
    //units::degree_t angle = GetAngleFromTurns(m_HoodMotor.GetPosition().GetValue()) - m_HoodOffset;
    return kFixedPositionHoodAngle;
};

units::degree_t ShooterSubsystem::GetAngleFromTurns(units::turn_t rotations) {
    units::degree_t angle = units::degree_t(rotations * kGearRatio);
    return angle;
}

units::turn_t ShooterSubsystem::GetTurnsFromAngle(units::degree_t angle) {
    units::turn_t rotations = units::turn_t((angle / kGearRatio));
    return rotations;
}

frc2::CommandPtr ShooterSubsystem::GoToAngle(units::degree_t angle) {
    angle = units::degree_t(std::clamp(angle.value(), 55.0, 75.0));
    return Run([this, angle] {
        //m_HoodMotor.SetControl(m_HoodPositionVoltage.WithPosition(GetTurnsFromAngle(angle + m_HoodOffset)));
    });
}

void ShooterSubsystem::SetGoals(units::revolutions_per_minute_t speed, units::degree_t hoodAngle) {
    m_FlywheelMotor.SetControl(m_ShooterVelocityVoltage.WithVelocity(speed));
    m_FlywheelFollowerMotor.SetControl(m_ShooterVelocityVoltage.WithVelocity(speed));
    m_FeederMotor.SetControl(m_FeederVelocityVoltage.WithVelocity(4500_rpm));

    // @todo Enable this when hood control is working
    // m_HoodMotor.SetControl(m_HoodPositionVoltage.WithPosition(GetTurnsFromAngle(hoodAngle + m_HoodOffset)));
}

units::revolutions_per_minute_t ShooterSubsystem::GetCurrentShooterSpeed() {
    units::revolutions_per_minute_t speed = m_FlywheelMotor.GetVelocity().GetValue();
    return speed;
};

frc2::CommandPtr ShooterSubsystem::StopShooter(){
    return RunOnce([this] {
        m_FlywheelMotor.SetControl(m_Stop);
        m_FlywheelFollowerMotor.SetControl(m_Stop);
        m_FeederMotor.SetControl(m_Stop);
    });
}

frc2::CommandPtr ShooterSubsystem::StopHood() {
    return RunOnce([this] {
        //m_HoodMotor.SetControl(m_Stop);
    });
}

frc2::CommandPtr ShooterSubsystem::EnableShooterWithHubTracking() {
    return Run([this] {
        TrajectoryInfo parameters = LaunchHelper::GetInstance().GetLaunchParameters();
        SetGoals(parameters.wheel_rpm, parameters.elevation_angle);
    });
}

frc2::CommandPtr ShooterSubsystem::EnableShooterWithFixedHoodAngle() {
    return Run([this] {
        TrajectoryInfo parameters = LaunchHelper::GetInstance().GetLaunchParameters();
        // SetGoals(parameters.wheel_rpm + 1500_rpm, kFixedPositionHoodAngle);
        SetGoals(parameters.wheel_rpm, kFixedPositionHoodAngle);
    });
}

frc2::CommandPtr ShooterSubsystem::EnableShooterWithFixedHoodAndFixedSpeed() {
    return Run([this] {
        SetGoals(4500_rpm, kFixedPositionHoodAngle);
    });
}

frc2::CommandPtr ShooterSubsystem::TestRunShooter() {
    return RunOnce([this] {
        m_FlywheelMotor.SetControl(m_ShooterVelocityVoltage.WithVelocity(1000_rpm));
    });
}

frc2::CommandPtr ShooterSubsystem::TestRunFeeder() {
    return RunOnce([this] {
        m_FeederMotor.SetControl(m_FeederVelocityVoltage.WithVelocity(1000_rpm));
    });
}

frc2::CommandPtr ShooterSubsystem::TestRunHoodAngle(units::degree_t angle) {
    return RunOnce([this, angle] {
        //m_HoodMotor.SetControl(m_HoodPositionVoltage.WithPosition(GetTurnsFromAngle(angle)));
    });
}

void ShooterSubsystem::SimulationInit() {
    const double kSimShooterElevationLineWidth = 6;
    m_ShooterElevationMech = m_MechRoot->Append<frc::MechanismLigament2d>("ShooterElevation", kShooterElevationRadius.value(), 0_deg, kSimShooterElevationLineWidth, frc::Color8Bit{frc::Color::kPurple});
    frc::SmartDashboard::PutData("ShooterElevation Sim", &m_Mech);

    //auto& shooterElevation_sim = m_HoodMotor.GetSimState();
    //shooterElevation_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    //shooterElevation_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

void ShooterSubsystem::SimulationPeriodic() {
    auto& flywheel_sim = m_FlywheelMotor.GetSimState();
    flywheel_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = flywheel_sim.GetMotorVoltage();
    m_FlywheelSimModel.SetInputVoltage(motor_voltage);

    auto& flywheel_follower_sim = m_FlywheelFollowerMotor.GetSimState();
    flywheel_follower_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    // Simulate the 20ms run in the simulation model
    m_FlywheelSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_FlywheelSimModel.GetCurrentDraw()}));

    flywheel_sim.SetRotorVelocity(m_FlywheelSimModel.GetAngularVelocity());
    flywheel_sim.SetRotorAcceleration(m_FlywheelSimModel.GetAngularAcceleration());


    //Shooter Elevation Sim
    //auto& shooterElevation_sim = m_HoodMotor.GetSimState();
    //shooterElevation_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    //auto sMotor_voltage = shooterElevation_sim.GetMotorVoltage();
    //m_ShooterElevationSimModel.SetInputVoltage(sMotor_voltage);

    // Simulate the 20ms run in the simulation model
    //m_ShooterElevationSimModel.Update(20_ms);

    //frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_ShooterElevationSimModel.GetCurrentDraw()}));

    // Update the simulated state for the shooterElevation motor
    //shooterElevation_sim.SetRawRotorPosition(kGearRatio * m_ShooterElevationSimModel.GetAngle());
    //shooterElevation_sim.SetRotorVelocity(kGearRatio * m_ShooterElevationSimModel.GetVelocity());

    // Update the simulated UI mechanism to the new angle based on the motor
    //m_ShooterElevationMech->SetAngle(GetCurrentHoodAngle());
}