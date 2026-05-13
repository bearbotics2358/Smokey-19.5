#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/button/Trigger.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <units/length.h>

#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>

using namespace ctre::phoenix6;

class ShooterSubsystem : public frc2::SubsystemBase {
public:
    ShooterSubsystem();

    units::revolutions_per_minute_t GetCurrentShooterSpeed();
    units::degree_t GetCurrentHoodAngle();
    void SetGoals(units::revolutions_per_minute_t speed, units::degree_t hoodAngle);
    void Periodic() override;
    void SimulationPeriodic() override;
    frc2::CommandPtr StopShooter();
    frc2::CommandPtr StopHood();
    frc2::CommandPtr EnableShooterWithHubTracking();
    frc2::CommandPtr EnableShooterWithFixedHoodAngle();
    frc2::CommandPtr EnableShooterWithFixedHoodAndFixedSpeed();
    frc2::CommandPtr GoToAngle(units::degree_t angle);

    frc2::CommandPtr TestRunShooter();
    frc2::CommandPtr TestRunFeeder();
    frc2::CommandPtr TestRunHoodAngle(units::degree_t angle);

    frc2::CommandPtr CalibrateHoodMotor();

    static constexpr units::degree_t kFixedPositionHoodAngle = 60_deg;

    units::degree_t m_HoodOffset = 75_deg;

    frc2::Trigger m_isHardStop;

private:
    void ConfigureShooterMotors();
    void ConfigureHoodMotor();
    void ConfigureFeederMotor();

    bool m_HoodZeroed = false;

    static constexpr int kFlywheelMotorId = 37;
    static constexpr int kFlywheelFollowerMotorId = 36;
    static constexpr int kShooterElevationMotorID = 50;
    static constexpr int kFeederMotorId = 30;

    static constexpr units::revolutions_per_minute_t kFixedPositionSpeed = 3600_rpm;

    hardware::TalonFX m_FlywheelMotor{kFlywheelMotorId};
    hardware::TalonFX m_FlywheelFollowerMotor{kFlywheelFollowerMotorId};
    hardware::TalonFX m_FeederMotor{kFeederMotorId};
    //hardware::TalonFX m_HoodMotor{kShooterElevationMotorID};

    controls::VelocityVoltage m_ShooterVelocityVoltage = controls::VelocityVoltage(0_rpm).WithSlot(0);
    controls::VelocityVoltage m_FeederVelocityVoltage = controls::VelocityVoltage(0_rpm).WithSlot(0);
    controls::PositionVoltage m_HoodPositionVoltage = controls::PositionVoltage(2.2_tr).WithSlot(0);
    controls::NeutralOut m_Stop;

    //Elevation Motor Inits
    units::degree_t GetAngleFromTurns(units::turn_t rotations);
    units::turn_t GetTurnsFromAngle(units::degree_t angle);

    static constexpr double kGearRatio = 1/28.435;

    controls::DutyCycleOut calibrationRequest = controls::DutyCycleOut(-0.1)
        .WithIgnoreHardwareLimits(true)
        .WithIgnoreSoftwareLimits(true);


    ////////////////////////////
    // Simulation related values
    //
    // Reduction between motors and encoder, as output over input. If the flywheel
    // spins slower than the motors, this number should be greater than one.
    static constexpr double kFlywheelGearing = 1.0;

    // @todo Figure out a real moment of inertia for the flywheel for a better simulation
    static constexpr units::kilogram_square_meter_t kFlywheelMomentOfInertia = 0.5 * 1.5_lb * 4_in * 4_in;

    frc::DCMotor m_ShooterGearbox{frc::DCMotor::KrakenX60(1)};
    frc::LinearSystem<1, 1, 1> m_Plant{frc::LinearSystemId::FlywheelSystem(
      m_ShooterGearbox, kFlywheelMomentOfInertia, kFlywheelGearing)};
    frc::sim::FlywheelSim m_FlywheelSimModel{m_Plant, m_ShooterGearbox};

    void SimulationInit();
    frc::Mechanism2d m_Mech{1, 1};
    frc::MechanismRoot2d* m_MechRoot{m_Mech.GetRoot("shooterElevationRoot", 0.5, 0.5)};
    frc::MechanismLigament2d* m_ShooterElevationMech;
    const units::meter_t kShooterElevationRadius = 12_in;
    frc::DCMotor m_ShooterElevationGearbox{frc::DCMotor::KrakenX60(1)};
    frc::sim::SingleJointedArmSim m_ShooterElevationSimModel{
      m_ShooterElevationGearbox,
      kGearRatio,
      frc::sim::SingleJointedArmSim::EstimateMOI(kShooterElevationRadius, 0.2_kg),
      kShooterElevationRadius,
      55_deg,
      75_deg,
      false,
      10_deg,
    };
};