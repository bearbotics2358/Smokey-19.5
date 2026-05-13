#include "subsystems/TurretSubsystem.h"

#include "bearlog/bearlog.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/util/Color8Bit.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include "subsystems/RobotZoneHelper.h"
#include "FieldConstants.h"

TurretSubsystem::TurretSubsystem(std::function<frc::Pose2d()> getBotPose)
    : m_turretSpinMotor(kTurretMotorID),
      m_GetCurrentBotPose(getBotPose)
{
    ctre::phoenix6::configs::Slot0Configs slot0Config;
    slot0Config
        .WithKP(5)
        .WithKI(0)
        .WithKD(0.2)
        .WithKS(0)
        .WithKV(0.2);

    // In this config object, we can also apply other things such as current limits,
    // brake mode, which direction is positive rotation, etc.
    ctre::phoenix6::configs::TalonFXConfiguration rotation_config;
    rotation_config
        .WithSlot0(slot0Config)
        .MotorOutput.WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        .WithPeakForwardDutyCycle(.1)
        .WithPeakReverseDutyCycle(.1);

    // Must apply the config to the motor during construction of this object and NOT within functions that
    // run in the normal Periodic loop
    m_turretSpinMotor.GetConfigurator().Apply(rotation_config);

    if (frc::RobotBase::IsSimulation()) {
        SimulationInit();
    }

    frc2::RobotModeTriggers::Autonomous().OnTrue(
        frc2::cmd::RunOnce([this] {
            if (false == m_TurretZeroedInit) {
                m_turretOffset = GetAngleFromTurns(m_turretSpinMotor.GetPosition().GetValue());
                m_TurretZeroedInit = true;
            }
        })
    );

    frc2::RobotModeTriggers::Teleop().OnTrue(
        frc2::cmd::RunOnce([this] {
            if (false == m_TurretZeroedInit) {
                m_turretOffset = GetAngleFromTurns(m_turretSpinMotor.GetPosition().GetValue());
                m_TurretZeroedInit = true;
            }
        })
    );

    m_Sensor = frc2::Trigger([this] {
        return m_turretReset.Get();
    });
}

void TurretSubsystem::Periodic() {
    BearLog::Log("Turret/Angle", CurrentAngle());
    BearLog::Log("Turret/position", m_turretSpinMotor.GetPosition().GetValue());
    BearLog::Log("Turret/offset", m_turretOffset);
    BearLog::Log("Turret/PoseRotation", m_GetCurrentBotPose().Rotation().Degrees());

    GoToAngle();
    BearLog::Log("Turret/Sensor", m_Sensor.Get());
    BearLog::Log("Turret/TurretOffset", m_turretOffset);
}

frc2::CommandPtr TurretSubsystem::NudgeOffsetUp() {
    return RunOnce([this] {
        m_turretOffset += 2_deg;
    });
}

frc2::CommandPtr TurretSubsystem::NudgeOffsetDown() {
    return RunOnce([this] {
        m_turretOffset -= 2_deg;
    });
}

units::degree_t TurretSubsystem::AngleToHub() {
    frc::Pose2d robotPose = m_GetCurrentBotPose();
    frc::Translation2d myHubPose = FieldConstants::GetHubCenterForMyAlliance();

    units::meter_t strafe = robotPose.Y() - myHubPose.Y();
    units::meter_t forward = robotPose.X() - myHubPose.X();
    units::degree_t robotAngle = robotPose.Rotation().Degrees();
    units::degree_t angleToHub;

    angleToHub = units::degree_t(units::radian_t(atan2(strafe.value(), forward.value())));

    BearLog::Log("Turret/RawSetpoint", angleToHub);
    angleToHub -= robotAngle;
    while (angleToHub.value() > 180) {
        angleToHub -= 360_deg;
    }
    while (angleToHub.value() < -180) {
        angleToHub += 360_deg;
    }
    BearLog::Log("Turret/ClampedSetpoint", angleToHub);
    return angleToHub;
}

units::degree_t TurretSubsystem::AngleToAllianceZone() {
    frc::Pose2d robotPose = m_GetCurrentBotPose();

    units::degree_t robotAngle = robotPose.Rotation().Degrees();
    units::degree_t angleToHub;

    angleToHub = 0_deg;

    BearLog::Log("Turret/RawSetpoint", angleToHub);
    angleToHub -= robotAngle;
    while (angleToHub.value() > 180) {
        angleToHub -= 360_deg;
    }
    while (angleToHub.value() < -180) {
        angleToHub += 360_deg;
    }

    return angleToHub;
}

frc2::CommandPtr TurretSubsystem::PointAtHub() {
    return frc2::cmd::RunOnce([this] {
        if (m_pointAtHubToggle == true) {
            m_pointAtHubToggle = false;
        } else {
            m_pointAtHubToggle = true;
        }
        BearLog::Log("Turret/PointToggle", m_pointAtHubToggle);
    });
}

frc2::CommandPtr TurretSubsystem::ZeroTurret() {
    return Run([this] {
        m_TurretZeroed = false;
        m_turretSpinMotor.SetControl(m_RotationVoltage.WithPosition(GetTurnsFromAngle(165_deg)));
    }).Until(
        [this] { return ((m_Sensor.Get() == false)); }
    ).WithTimeout(0.5_s).AndThen(
        Run([this] {
            m_turretSpinMotor.SetControl(5_V);
        }).Until(
            [this] { return ((m_Sensor.Get())); }
        )
    ).AndThen(
        Run([this] {
            m_turretSpinMotor.SetControl(-2_V);
        }).Until(
            [this] { return ((m_Sensor.Get() == false) || (CurrentAngle() < 0_deg)); }
        )
    ).AndThen(
        RunOnce([this] {
            m_turretSpinMotor.SetVoltage(0_V);
            m_turretOffset = GetAngleFromTurns(m_turretSpinMotor.GetPosition().GetValue()) - 91.5_deg;
            m_TurretZeroed = true;
        })
    );
}

void TurretSubsystem::SetGoalAngle() {
    if (m_pointAtHubToggle == false) {
        m_setpointAngle = m_stowAngle;
    } else {
        if (RobotZoneHelper::isRobotInMyAllianceZone(m_GetCurrentBotPose())) {
            m_setpointAngle = AngleToHub();
        } else if (RobotZoneHelper::isRobotInNeutralZone(m_GetCurrentBotPose())) {
            m_setpointAngle = AngleToAllianceZone();
        }
    }
}

units::degree_t TurretSubsystem::CurrentAngle() {
    units::degree_t angle = GetAngleFromTurns(m_turretSpinMotor.GetPosition().GetValue()) - m_turretOffset;
    return angle;
};

units::degree_t TurretSubsystem::GetAngleFromTurns(units::turn_t rotations) {
    units::degree_t angle = rotations / kGearRatio;
    return angle;
}

units::turn_t TurretSubsystem::GetTurnsFromAngle(units::degree_t angle) {
    units::turn_t rotations = angle * kGearRatio;
    return rotations;
}

void TurretSubsystem::GoToAngle() {
    SetGoalAngle();
    units::turn_t position_in_motor_turns = GetTurnsFromAngle(m_setpointAngle + m_turretOffset);


    BearLog::Log("aTurretZeroed", m_TurretZeroed);
    if (m_TurretZeroed == true) {
        m_turretSpinMotor.SetControl(
            m_RotationVoltage.WithPosition(position_in_motor_turns)
            .WithSlot(0));
    }
}

// Runs in Simulation only!
void TurretSubsystem::SimulationInit() {
    const double kSimTurretLineWidth = 6;
    m_TurretMech = m_MechRoot->Append<frc::MechanismLigament2d>("Turret", kTurretRadius.value(), 0_deg, kSimTurretLineWidth, frc::Color8Bit{frc::Color::kPurple});
    frc::SmartDashboard::PutData("Turret Sim", &m_Mech);

    auto& turret_sim = m_turretSpinMotor.GetSimState();
    turret_sim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    turret_sim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}

// Runs in Simulation only!
void TurretSubsystem::SimulationPeriodic() {
    auto& turret_sim = m_turretSpinMotor.GetSimState();
    turret_sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    auto motor_voltage = turret_sim.GetMotorVoltage();
    m_TurretSimModel.SetInputVoltage(motor_voltage);

    // Simulate the 20ms run in the simulation model
    m_TurretSimModel.Update(20_ms);

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({m_TurretSimModel.GetCurrentDraw()}));

    // Update the simulated state for the turret motor
    turret_sim.SetRawRotorPosition(kGearRatio * m_TurretSimModel.GetAngle());
    turret_sim.SetRotorVelocity(kGearRatio * m_TurretSimModel.GetVelocity());

    // Update the simulated UI mechanism to the new angle based on the motor
    m_TurretMech->SetAngle(CurrentAngle());

    // AdvantageScope does not yet support a mechanism that is in the XY plane. To visualize that in 3d mode,
    // we construct a Pose3d object and use a Cone object in AdvantageScope.
    frc::Pose2d bot_pose = m_GetCurrentBotPose();

    // Incorporate the rotation of the robot to simulate the turret being attached
    frc::Pose3d turret_sim_3d_pose = {bot_pose.X(), bot_pose.Y(), 12_in, frc::Rotation3d{0_rad, 0_rad, bot_pose.Rotation().Degrees() + CurrentAngle()}};
    BearLog::Log("Turret/Pose", turret_sim_3d_pose);
}