#include "subsystems/IndexerSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

IndexerSubsystem::IndexerSubsystem():
    m_indexerSpinMotor{kIndexerSpinMotorID}
{
    ctre::phoenix6::configs::TalonFXConfiguration configs{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 100_A;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;

    configs.CurrentLimits.StatorCurrentLimit = 60_A;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    configs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    configs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

    configs.Slot0.kP = 0.4;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kV = 0.8;

    m_indexerSpinMotor.GetConfigurator().Apply(configs);

    m_isHardStop = frc2::Trigger([this] {
        return abs(m_indexerSpinMotor.GetVelocity().GetValue().value()) < 1 &&
            abs(m_indexerSpinMotor.GetTorqueCurrent().GetValue().value()) > 30;
    }).Debounce(0.1_s);
}

frc2::CommandPtr IndexerSubsystem::RunIndexerForLaunching() {
    return Run([this] {
        m_indexerSpinMotor.SetControl(m_IndexerRequest.WithVelocity(750_rpm));
    }).Until(
        [this] { return m_isHardStop.Get(); }
    ).AndThen(RunIndexerInReverse())
    .Repeatedly();
}

frc2::CommandPtr IndexerSubsystem::RunIndexerInReverse() {
    return Run([this] {
        m_indexerSpinMotor.SetControl(m_IndexerRequest.WithVelocity(-200_rpm));
    }).WithTimeout(1_s)
    .AndThen(Stop());
}

frc2::CommandPtr IndexerSubsystem::Stop() {
    return RunOnce([this] {
        m_indexerSpinMotor.SetControl(m_Stop);
    });
}

void IndexerSubsystem::Periodic() {
    BearLog::Log("Indexer/MotorVelocity", GetMotorVelocity());
    BearLog::Log("Indexer/HardStop?", m_isHardStop.Get());
    BearLog::Log("Indexer/TorqueCurrent", m_indexerSpinMotor.GetTorqueCurrent().GetValue());
    BearLog::Log("Indexer/SupplyCurrent", m_indexerSpinMotor.GetStatorCurrent().GetValue());
}

units::revolutions_per_minute_t IndexerSubsystem::GetMotorVelocity() {
    units::revolutions_per_minute_t velocity = m_indexerSpinMotor.GetVelocity().GetValue();
    return velocity;
}

// Runs in Simulation only!
void IndexerSubsystem::SimulationPeriodic() {
    auto& indexer_sim = m_indexerSpinMotor.GetSimState();
    indexer_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = indexer_sim.GetMotorVoltage();
    m_IndexerSimModel.SetInputVoltage(motor_voltage);

    // Simulate the 20ms run in the simulation model
    m_IndexerSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_IndexerSimModel.GetCurrentDraw()}));

    indexer_sim.SetRotorVelocity(m_IndexerSimModel.GetAngularVelocity());
    indexer_sim.SetRotorAcceleration(m_IndexerSimModel.GetAngularAcceleration());
}