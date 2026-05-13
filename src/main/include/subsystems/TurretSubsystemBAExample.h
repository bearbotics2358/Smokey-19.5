//This was added by Shaan and pasted in from the email sent by Mr. Atchinson. 

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

using namespace ctre::phoenix6;

class TurretSubsystemExample : public frc2::SubsystemBase {

public:
  TurretSubsystemExample();

  void Periodic() override;

  void SetTurretAngle(units::degree_t angle);

private:
  units::turn_t GetMotorTurnsFromAngle(units::degree_t angle);
  units::degree_t GetAngleFromMotorTurns(units::turn_t turns);

private:
  const int kForwardLimitSwitchPort = 0;
  const int kRotationMotorCanId = 5;
  const int kGearRatio = 1;

  frc::DigitalInput m_ForwardLimit{kForwardLimitSwitchPort};
  hardware::TalonFX m_RotationMotor{kRotationMotorCanId};

  units::degree_t m_RotationSetPoint{0_deg};
  controls::PositionDutyCycle m_RotationDutyCycle{0_tr};
};
