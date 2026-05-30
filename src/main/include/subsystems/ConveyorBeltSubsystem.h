#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/Commands.h>

using namespace ctre::phoenix6;

class ConveyorBeltSubsystem : public frc2::SubsystemBase {
public:
    ConveyorBeltSubsystem();

    void Periodic() override;

    frc2::CommandPtr EnableBelt();
    frc2::CommandPtr Stop();
private:
    controls::VelocityVoltage m_BeltVoltage = controls::VelocityVoltage(0_rpm).WithSlot(0);
    controls::NeutralOut m_Stop;

    void ConfigureBeltMotor();

    static constexpr int kBeltMotorID = 62;
    hardware::TalonFX m_beltMotor{kBeltMotorID};
};