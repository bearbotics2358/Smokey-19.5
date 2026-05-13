#include "subsystems/IntakeSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

IntakeSubsystem::IntakeSubsystem()
{
    ConfigureIntakeMotor();

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }
}

void IntakeSubsystem::ConfigureIntakeMotor() {
    configs::TalonFXConfiguration configs{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 70_A;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;

    configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    configs.CurrentLimits.StatorCurrentLimit = 60_A;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    configs.Slot0.kP = 0.5;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kV = 0.12;

    m_intakeSpinMotor.GetConfigurator().Apply(configs);
}

void IntakeSubsystem::Periodic() {
    BearLog::Log("Intake/Velocity", units::revolutions_per_minute_t(m_intakeSpinMotor.GetVelocity().GetValue()));
}

frc2::CommandPtr IntakeSubsystem::RunIntake() {
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(2000_rpm));
    });
}

frc2::CommandPtr IntakeSubsystem::RunIntakeToHelpIndexer() {
    // Use this command when attempting to push more fuel into the indexer
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(500_rpm));
    });
}

frc2::CommandPtr IntakeSubsystem::RunIntakeInReverse() {
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(-2000_rpm));
    });
}

frc2::CommandPtr IntakeSubsystem::StopIntake() {
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_Stop);
    });
}

// Runs in Simulation only!
void IntakeSubsystem::SimulationInit() {
    auto& intake_sim = m_intakeSpinMotor.GetSimState();
    intake_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    intake_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

// Runs in Simulation only!
void IntakeSubsystem::SimulationPeriodic() {
    auto& intake_sim = m_intakeSpinMotor.GetSimState();
    intake_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = intake_sim.GetMotorVoltage();
    m_IntakeSimModel.SetInputVoltage(motor_voltage);

    // Simulate the 20ms run in the simulation model
    m_IntakeSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_IntakeSimModel.GetCurrentDraw()}));

    // Update the simulated state for the Intake motor
    intake_sim.SetRawRotorPosition(kGearRatio * m_IntakeSimModel.GetAngle());
    intake_sim.SetRotorVelocity(kGearRatio * m_IntakeSimModel.GetVelocity());
}