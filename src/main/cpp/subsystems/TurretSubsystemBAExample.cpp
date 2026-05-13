//This was added by Shaan and pasted in from the email sent by Mr. Atchinson. 

#include "subsystems/TurretSubsystemBAExample.h"

//#include "bearlog/bearlog.h"

using namespace ctre::phoenix6;

TurretSubsystemExample::TurretSubsystemExample()
{
  // Define a configuration for the rotation motor that will enable the forward limit and set the encoder
  // position to 0 turns when the limit switch is triggered
  configs::HardwareLimitSwitchConfigs rotation_limit_config;
  rotation_limit_config
    .WithForwardLimitAutosetPositionEnable(true)
    .WithForwardLimitEnable(true)
    .WithForwardLimitAutosetPositionValue(0_tr);

  // @todo Update these with real PID values for the motor
  configs::Slot0Configs slot0Config;
  slot0Config
    .WithKP(2)
    .WithKI(0)
    .WithKD(0)
    .WithKS(0)
    .WithKV(0.2);

  // Define the overall configuration with the new hardware limit switch config
  // In this config object, we can also apply other things such as current limits,
  // brake mode, which direction is positive rotation, etc.
  configs::TalonFXConfiguration rotation_config;
  rotation_config
    .WithHardwareLimitSwitch(rotation_limit_config)
    .WithSlot0(slot0Config);

  // Must apply the config to the motor during construction of this object and NOT within functions that
  // run in the normal Periodic loop
  m_RotationMotor.GetConfigurator().Apply(rotation_config);
}

units::turn_t TurretSubsystemExample::GetMotorTurnsFromAngle(units::degree_t angle)
{
  units::turn_t turns = units::turn_t((angle.value() / kGearRatio) / 360);
  return turns;
}

units::degree_t TurretSubsystemExample::GetAngleFromMotorTurns(units::turn_t turns)
{
  units::degree_t degrees = units::degree_t(turns.value() * 360 * kGearRatio);
  return degrees;
}

void TurretSubsystemExample::SetTurretAngle(units::degree_t angle)
{
  m_RotationSetPoint = angle;
  units::turn_t position_in_motor_turns = GetMotorTurnsFromAngle(angle);

  m_RotationMotor.SetControl(
    m_RotationDutyCycle.WithPosition(position_in_motor_turns)
      .WithLimitForwardMotion(m_ForwardLimit.Get())
      .WithSlot(0));
}

void TurretSubsystemExample::Periodic()
{
  units::degree_t turret_angle = GetAngleFromMotorTurns(m_RotationMotor.GetPosition().GetValue());
  // BearLog::Log("Turret/CurrentPosition", turret_angle);
  // BearLog::Log("Turret/SetPoint", m_RotationSetPoint);
}