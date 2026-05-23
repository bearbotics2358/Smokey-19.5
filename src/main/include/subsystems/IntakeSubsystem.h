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

class IntakeSubsystem : public frc2::SubsystemBase {
public:
        IntakeSubsystem();
        frc2::CommandPtr RunIntake();
        frc2::CommandPtr RunIntakeToHelpIndexer();
        frc2::CommandPtr RunIntakeInReverse();
        frc2::CommandPtr StopIntake();

        void Periodic() override;
        void SimulationPeriodic() override;
private:
        controls::VelocityVoltage m_IntakeVelocity = controls::VelocityVoltage(0_rpm).WithSlot(0);
        controls::NeutralOut m_Stop;

        void ConfigureIntakeMotor();

        static constexpr int kIntakeMotorID = 62;
        hardware::TalonFX m_intakeSpinMotor{kIntakeMotorID};

        static constexpr double kGearRatio = 1;

        void SimulationInit();
        const units::meter_t kIntakeRadius = 1_in;
        frc::DCMotor m_IntakeGearbox{frc::DCMotor::KrakenX60(1)};
        frc::sim::SingleJointedArmSim m_IntakeSimModel{
          m_IntakeGearbox,
          kGearRatio,
          frc::sim::SingleJointedArmSim::EstimateMOI(kIntakeRadius, 0.1_kg),
          kIntakeRadius,
          -180_deg,
          180_deg,
          false,
          10_deg,
        };
};