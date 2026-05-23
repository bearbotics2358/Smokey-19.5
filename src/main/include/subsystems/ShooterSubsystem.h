#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/Commands.h>

#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>

#include <units/length.h>

#include <frc2/command/button/Trigger.h>

using namespace ctre::phoenix6;

class ShooterSubsystem : public frc2::SubsystemBase {
public:
  ShooterSubsystem();
  units::revolutions_per_minute_t GetDrumSpeed();
  units::revolutions_per_minute_t CalculateFeederSpeed(); //as conveyor belts move up, speed up the feeders
  void SetGoals(units::revolutions_per_minute_t speed);
  void Periodic() override;
  void SimulationPeriodic() override;
  frc2::CommandPtr EnableShooter();
  frc2::CommandPtr DisableShooter();
  frc2::CommandPtr EnableDrum();//for spinning up
  frc2::CommandPtr SlowDrum(); //drum should not need to completely stop while robot is enabled
private:
  void ConfigureDrumMotors();
  void ConfigureFeederMotors();

  static constexpr int kDrumAMotorId = 1;//@todo: change to actual motor IDs
  static constexpr int kDrumBMotorId = 2;
  static constexpr int kDrumCMotorId = 3;
  static constexpr int kDrumDMotorId = 4;

  static constexpr int kFeederAMotorId = 5;
  static constexpr int kFeederBMotorId = 6;

  static constexpr units::revolutions_per_minute_t kSlowDrumSpeed = 100_rpm;

  hardware::TalonFX m_DrumAMotor{kDrumAMotorId};
  hardware::TalonFX m_DrumBMotor{kDrumBMotorId};
  hardware::TalonFX m_DrumCMotor{kDrumCMotorId};
  hardware::TalonFX m_DrumDMotor{kDrumDMotorId};

  hardware::TalonFX m_FeederAMotor{kFeederAMotorId};
  hardware::TalonFX m_FeederBMotor{kFeederBMotorId};

  controls::VelocityVoltage m_DrumVelocityVoltage = controls::VelocityVoltage(0_rpm).WithSlot(0);
  controls::VelocityVoltage m_FeederVelocityVoltage = controls::VelocityVoltage(0_rpm).WithSlot(0);
  controls::NeutralOut m_Stop;

};