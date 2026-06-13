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
  enum class EnableFeeder {No, Yes};

  ShooterSubsystem();
  units::revolutions_per_minute_t GetDrumSpeed();
  units::revolutions_per_minute_t GetFeederSpeed(); //as conveyor belts move up, speed up the feeders
  void SetGoalSpeeds(units::revolutions_per_minute_t speed);
  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr RunDrumAndFeeder();
  frc2::CommandPtr DisableDrumAndFeeder();
  frc2::CommandPtr RunDrumSlowly();

  frc2::CommandPtr TestDrum();
  frc2::CommandPtr TestFeeder();
  frc2::CommandPtr TestRollerBed();

  frc2::CommandPtr IncreaseDrumRPM();
  frc2::CommandPtr DecreaseDrumRPM();

private:
  units::revolutions_per_minute_t m_rpmOffset;

  void ConfigureDrumMotors();
  void ConfigureFeederMotors();

  void StopDrumMotors();
  void StopFeederMotors();
  void SetGoalSpeeds(units::revolutions_per_minute_t drumSpeed, EnableFeeder enableFeeder);

  static constexpr int kDrumAMotorId = 34;
  static constexpr int kDrumBMotorId = 33;
  static constexpr int kDrumCMotorId = 35;
  static constexpr int kDrumDMotorId = 36;

  static constexpr int kFeederAMotorId = 30;
  static constexpr int kFeederBMotorId = 32;

  static constexpr int kRollerBedMotorId = 28;

  hardware::TalonFX m_DrumAMotor{kDrumAMotorId};
  hardware::TalonFX m_DrumBMotor{kDrumBMotorId};
  hardware::TalonFX m_DrumCMotor{kDrumCMotorId};
  hardware::TalonFX m_DrumDMotor{kDrumDMotorId};

  hardware::TalonFX m_FeederAMotor{kFeederAMotorId};
  hardware::TalonFX m_FeederBMotor{kFeederBMotorId};

  hardware::TalonFX m_RollerBedMotor{kRollerBedMotorId};

  controls::VelocityVoltage m_DrumVelocityVoltage = controls::VelocityVoltage(0_rpm).WithSlot(0);
  controls::VelocityVoltage m_FeederVelocityVoltage = controls::VelocityVoltage(0_rpm).WithSlot(0);
  controls::VelocityVoltage m_RollerBedVelocityVoltage = controls::VelocityVoltage(0_rpm).WithSlot(0);
  controls::NeutralOut m_Stop;

};