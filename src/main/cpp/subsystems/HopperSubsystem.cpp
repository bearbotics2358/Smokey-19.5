#include "subsystems/HopperSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

HopperSubsystem::HopperSubsystem()
{
    ConfigureExtenderMotor();
    ConfigureExtenderCANCoder();

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }

    m_ExtenderHardStop = frc2::Trigger([this] {
        return (units::math::abs(m_extenderMotor.GetVelocity().GetValue()) < 1_tps &&
            units::math::abs(m_extenderMotor.GetTorqueCurrent().GetValue()) > 30_A);
    }).Debounce(0.1_s);
}

void HopperSubsystem::ConfigureExtenderMotor() {
    configs::TalonFXConfiguration extender_config{};

    static constexpr units::ampere_t kPeakTorqueCurrent = 70_A;
    extender_config.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueCurrent;
    extender_config.TorqueCurrent.PeakReverseTorqueCurrent = -kPeakTorqueCurrent;
    extender_config.CurrentLimits.StatorCurrentLimit = 50_A;
    extender_config.CurrentLimits.StatorCurrentLimitEnable = true;

    extender_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    extender_config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    extender_config.Slot0.kP = 30.0;
    extender_config.Slot0.kI = 0.0;
    extender_config.Slot0.kD = 0.0;
    extender_config.Slot0.kV = 0.12;

    extender_config.MotionMagic.MotionMagicCruiseVelocity = 50_tps;
    extender_config.MotionMagic.MotionMagicAcceleration = 160_tr_per_s_sq;

    extender_config.Feedback.FeedbackRemoteSensorID = m_ExtenderCANCoder.GetDeviceID();
    extender_config.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
    extender_config.Feedback.RotorToSensorRatio = 8.1818181818;
    extender_config.Feedback.SensorToMechanismRatio = 1.0;

    m_extenderMotor.GetConfigurator().Apply(extender_config);
}

void HopperSubsystem::ConfigureExtenderCANCoder() {
    configs::CANcoderConfiguration config{};

    config.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = -0.213135_tr;

    m_ExtenderCANCoder.GetConfigurator().Apply(config);
}

void HopperSubsystem::Periodic() {
    BearLog::Log("Intake/Extender/Angle", CurrentAngle());
    BearLog::Log("Intake/Extender/Turns", m_extenderMotor.GetPosition().GetValue());
    BearLog::Log("Intake/Extender/Setpoint", GetAngleFromTurns(m_ExtenderVoltage.Position));
    BearLog::Log("Intake/Extender/Current", m_extenderMotor.GetTorqueCurrent().GetValue());
    BearLog::Log("Intake/Extender/Voltage", m_extenderMotor.GetMotorVoltage().GetValue());

    BearLog::Log("Intake/Extender/CANcoder position", m_ExtenderCANCoder.GetPosition().GetValue());
    BearLog::Log("Intake/Extender/Setpoint", m_ExtenderVoltage.Position);
}

frc2::CommandPtr HopperSubsystem::AgitateToHelpIndexer() {
    return AgitateIn()
    .AndThen(AgitateOut())
    .Repeatedly()
    .AndThen(StopHopper());
}

frc2::CommandPtr HopperSubsystem::AgitateIn(){
    return Run([this]{
        m_extenderMotor.SetControl(m_ExtenderVoltage.WithPosition(0.05_tr));
    }).WithTimeout(0.5_s);
}

frc2::CommandPtr HopperSubsystem::AgitateOut(){
    return Run([this]{
        m_extenderMotor.SetControl(m_ExtenderVoltage.WithPosition(0.25_tr));
    }).WithTimeout(0.5_s);
}

frc2::CommandPtr HopperSubsystem::ExtendExtenderConstantVolts() {
    return Run([this] {
        m_extenderMotor.SetVoltage(2.0_V);
    });
}

frc2::CommandPtr HopperSubsystem::RetractExtenderConstantVolts() {
    return Run([this] {
        m_extenderMotor.SetVoltage(-2.0_V);
    });
}

units::degree_t HopperSubsystem::CurrentAngle() {
    units::degree_t angle = GetAngleFromTurns(m_extenderMotor.GetPosition().GetValue()) * kEGearRatio;
    return angle;
}

frc2::CommandPtr HopperSubsystem::ExtendHopper() {
    return Run([this] {
        m_extenderMotor.SetControl(m_ExtenderVoltage.WithPosition(0.28_tr).WithSlot(0));
    }).Until(
        // This could probably be done using WithLimitForwardMotion, but this works for now
        [this] { return m_ExtenderHardStop.Get(); }
    ).AndThen(
        StopHopper()
    );
}

frc2::CommandPtr HopperSubsystem::StowHopper() {
    return Run([this] {
        m_extenderMotor.SetControl(m_ExtenderVoltage.WithPosition(0_tr).WithSlot(0));
    }).Until(
        // This could probably be done using WithLimitForwardMotion, but this works for now
        [this] { return m_ExtenderHardStop.Get(); }
    ).AndThen(
        StopHopper()
    );
}

frc2::CommandPtr HopperSubsystem::StopHopper() {
    return RunOnce([this] {
        m_extenderMotor.SetControl(m_Stop);
    });
}

units::degree_t HopperSubsystem::GetAngleFromTurns(units::turn_t rotations) {
    units::degree_t angle = units::degree_t(rotations.value() * kEGearRatio);
    return angle;
}

units::turn_t HopperSubsystem::GetTurnsFromAngle(units::degree_t angle) {
    units::turn_t rotations = units::turn_t(angle.value() / kEGearRatio);
    return rotations;
}

// Runs in Simulation only!
void HopperSubsystem::SimulationInit() {
    const double kSimIntakeLineWidth = 6;

    m_EIntakeMech = m_EMechRoot->Append<frc::MechanismLigament2d>("Turret", kEIntakeRadius.value(), 0_deg, kSimIntakeLineWidth, frc::Color8Bit{frc::Color::kPurple});
    frc::SmartDashboard::PutData("Extender Sim", &m_EMech);

    auto& extender_sim = m_extenderMotor.GetSimState();
    extender_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    extender_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

// Runs in Simulation only!
void HopperSubsystem::SimulationPeriodic() {
    auto& extender_sim = m_extenderMotor.GetSimState();
    extender_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto eMotor_voltage = extender_sim.GetMotorVoltage();
    m_EIntakeSimModel.SetInputVoltage(eMotor_voltage);

    // Simulate the 20ms run in the simulation model
    m_EIntakeSimModel.Update(20_ms);

    // Update the simulated state for the Intake motor
    extender_sim.SetRawRotorPosition(kEGearRatio * m_EIntakeSimModel.GetAngle());
    extender_sim.SetRotorVelocity(kEGearRatio * m_EIntakeSimModel.GetVelocity());

    // Update the simulated UI mechanism to the new angle based on the motor
    m_EIntakeMech->SetAngle(CurrentAngle());
}