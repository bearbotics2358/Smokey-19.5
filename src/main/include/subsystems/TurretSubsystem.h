#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/geometry/Pose3d.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/Encoder.h>

#include <units/length.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/button/Trigger.h>

#include <frc/DigitalInput.h>

constexpr int kTurretMotorID = 60;

using namespace ctre::phoenix6;

class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem(std::function<frc::Pose2d()> getBotPose);

        void SetGoalAngle();
        units::degree_t CurrentAngle();

        units::degree_t AngleToHub();
        units::degree_t AngleToAllianceZone();

        frc2::CommandPtr PointAtHub();
        frc2::CommandPtr NudgeOffsetUp();
        frc2::CommandPtr NudgeOffsetDown();

        frc2::CommandPtr ZeroTurret();

        void Periodic() override;
        void SimulationPeriodic() override;

        bool m_pointAtHubToggle = true;

        units::degree_t m_stowAngle = 0_deg;

        frc2::Trigger m_Sensor;
    private:
        void GoToAngle();
        units::degree_t GetAngleFromTurns(units::turn_t rotations);
        units::turn_t GetTurnsFromAngle(units::degree_t angle);

        frc::DigitalInput m_turretReset{0};
        units::degree_t m_turretOffset = 0_deg;

        bool m_TurretZeroedInit = false;
        bool m_TurretZeroed = true;

        controls::PositionVoltage m_RotationVoltage = controls::PositionVoltage(0_tr).WithSlot(0);

        ctre::phoenix6::hardware::TalonFX m_turretSpinMotor;
        std::function<frc::Pose2d()> m_GetCurrentBotPose;

        static constexpr double kGearRatio = 40;

        units::degree_t m_setpointAngle = 0_deg;

        // Simulation specific items
        void SimulationInit();
        frc::Mechanism2d m_Mech{1, 1};
        frc::MechanismRoot2d* m_MechRoot{m_Mech.GetRoot("turretRoot", 0.5, 0.5)};
        frc::MechanismLigament2d* m_TurretMech;
        const units::meter_t kTurretRadius = 12_in;
        frc::DCMotor m_TurretGearbox{frc::DCMotor::KrakenX60(1)};
        frc::sim::SingleJointedArmSim m_TurretSimModel{
          m_TurretGearbox,
          kGearRatio,
          frc::sim::SingleJointedArmSim::EstimateMOI(kTurretRadius, 0.1_kg),
          kTurretRadius,
          -360_deg,
          360_deg,
          false,
          10_deg,
        };

};