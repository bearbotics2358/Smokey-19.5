#include "subsystems/ConveyorBeltSubsystem.h"

#include "bearlog/bearlog.h"
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc2/command/button/RobotModeTriggers.h>

using namespace ctre::phoenix6;

ConveyorBeltSubsystem::ConveyorBeltSubsystem()
{
    ConfigureBeltMotor();

    // Be sure to stop all the motors if the robot is disabled while it's running
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        Stop().IgnoringDisable(true)
    );
}

void ConveyorBeltSubsystem::ConfigureBeltMotor() {
    configs::TalonFXConfiguration config{};

    // @todo Find out if this is a good stator current limit. It may need to be higher or lower depending on testing.
    config.CurrentLimits.StatorCurrentLimit = 80_A;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // @todo Find out if this is a good supply current limit. It may need to be higher or lower depending on testing.
    config.CurrentLimits.SupplyCurrentLimit = 50_A;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    config.Slot0.kP = 30.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12;

    m_beltMotor.GetConfigurator().Apply(config);
}

void ConveyorBeltSubsystem::Periodic() {
    BearLog::Log("Conveyor/Belt/Speed", units::revolutions_per_minute_t(m_beltMotor.GetVelocity().GetValue()));
    BearLog::Log("Conveyor/Belt/SpeedSetPoint", units::revolutions_per_minute_t(m_BeltVoltage.Velocity()));
}

frc2::CommandPtr ConveyorBeltSubsystem::TestBelt() {
    return Run([this] {
        m_beltMotor.SetVoltage(4_V);
    });
}

frc2::CommandPtr ConveyorBeltSubsystem::RunBelt() {
    return RunOnce([this] {
        // @todo Re-enable this when the conveyor belt is fixed!
        // m_beltMotor.SetControl(m_BeltVoltage.WithVelocity(1500_rpm));
    });
}

frc2::CommandPtr ConveyorBeltSubsystem::Stop() {
    return RunOnce([this] {
        m_beltMotor.SetControl(m_Stop);
    });
}
