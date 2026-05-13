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

class HopperSubsystem : public frc2::SubsystemBase {
public:
    HopperSubsystem();
    frc2::CommandPtr StopIntake();
    frc2::CommandPtr AgitateToHelpIndexer();
    frc2::CommandPtr AgitateIn();
    frc2::CommandPtr AgitateOut();

    frc2::CommandPtr ExtendExtenderConstantVolts();
    frc2::CommandPtr RetractExtenderConstantVolts();
    units::degree_t CurrentAngle();

    void Periodic() override;
    void SimulationPeriodic() override;

    frc2::CommandPtr SetGoalAngle();
    frc2::CommandPtr ExtendHopper();
    frc2::CommandPtr StowHopper();
    frc2::CommandPtr StopHopper();
private:
    controls::MotionMagicVoltage m_ExtenderVoltage = controls::MotionMagicVoltage(0_tr).WithSlot(0);
    controls::NeutralOut m_Stop;

    units::degree_t GetAngleFromTurns(units::turn_t rotations);
    units::turn_t GetTurnsFromAngle(units::degree_t angle);

    void ConfigureExtenderMotor();
    void ConfigureExtenderCANCoder();

    static constexpr int kExtenderMotorID = 61;
    static constexpr int kExtenderCANCoderID = 33;
    hardware::TalonFX m_extenderMotor{kExtenderMotorID};
    hardware::CANcoder m_ExtenderCANCoder{kExtenderCANCoderID};

    static constexpr double kEGearRatio = 1;
    bool isExtended = false;

    frc2::Trigger m_ExtenderHardStop;

    void SimulationInit();

    frc::Mechanism2d m_EMech{1, 1};
    frc::MechanismRoot2d* m_EMechRoot{m_EMech.GetRoot("extenderRoot", 0.5, 0.5)};
    frc::MechanismLigament2d* m_EIntakeMech;
    const units::meter_t kEIntakeRadius = 12_in;
    frc::DCMotor m_EIntakeGearbox{frc::DCMotor::KrakenX60(1)};
    frc::sim::SingleJointedArmSim m_EIntakeSimModel{
        m_EIntakeGearbox,
        kEGearRatio,
        frc::sim::SingleJointedArmSim::EstimateMOI(kEIntakeRadius, 0.1_kg),
        kEIntakeRadius,
        -180_deg,
        180_deg,
        false,
        10_deg,
    };
};