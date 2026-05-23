#pragma once

#include <vector>
#include <memory>
#include <span>
#include <initializer_list>

#include <frc2/command/SubsystemBase.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "vision/IVisionInput.h"

typedef std::function<void(frc::Pose2d, units::second_t, std::array<double, 3>)> VisionMeasurementCallback;

class VisionSubsystem : public frc2::SubsystemBase {
public:
  VisionSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain, std::vector<std::unique_ptr<IVisionInput>> cameras);

  void Periodic() override;

private:
  subsystems::CommandSwerveDrivetrain* m_Drivetrain;
  std::vector<std::unique_ptr<IVisionInput>> m_LocalizationCameras;
  std::vector<IVisionInput::Inputs> m_Inputs;
};