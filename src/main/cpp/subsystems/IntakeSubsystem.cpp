#include "subsystems/IntakeSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/RobotModeTriggers.h>

using namespace ctre::phoenix6;

IntakeSubsystem::IntakeSubsystem()
{
    ConfigureIntakeMotor();

    m_IntakeHardStop = frc2::Trigger([this] {
        return (units::math::abs(m_intakeSpinMotor.GetVelocity().GetValue()) < 1_tps &&
            units::math::abs(m_intakeSpinMotor.GetTorqueCurrent().GetValue()) > 100_A);
    }).Debounce(0.1_s);

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }

    // Be sure to stop all the motors if the robot is disabled while it's running
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        StopIntake().IgnoringDisable(true)
    );
}

void IntakeSubsystem::ConfigureIntakeMotor() {
    configs::TalonFXConfiguration configs{};

    // @todo Find out if this is a good stator current limit. It may need to be higher or lower depending on testing.
    configs.CurrentLimits.StatorCurrentLimit = 60_A;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    // @todo Find out if this is a good supply current limit. It may need to be higher or lower depending on testing.
    configs.CurrentLimits.SupplyCurrentLimit = 30_A;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    configs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    configs.Slot0.kP = 0.5;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kV = 0.12;

    m_intakeSpinMotor.GetConfigurator().Apply(configs);
}

void IntakeSubsystem::Periodic() {
    BearLog::Log("Intake/Velocity", units::revolutions_per_minute_t(m_intakeSpinMotor.GetVelocity().GetValue()));
}

frc2::CommandPtr IntakeSubsystem::TestIntake() {
    return Run([this] {
        m_intakeSpinMotor.SetVoltage(-2_V);
    });
}

frc2::CommandPtr IntakeSubsystem::RunIntake() {
    return RunOnce([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(2000_rpm));
    });
}

frc2::CommandPtr IntakeSubsystem::RunIntakeJamProtection() {
    return Run([this] {
        m_intakeSpinMotor.SetControl(m_IntakeVelocity.WithVelocity(2000_rpm));
    }).Until(
        [this] { return m_IntakeHardStop.Get(); }
    ).AndThen(
        RunIntakeInReverse().Repeatedly().Until([this] { return m_IntakeHardStop.Get() == false; })
    ).Repeatedly();
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