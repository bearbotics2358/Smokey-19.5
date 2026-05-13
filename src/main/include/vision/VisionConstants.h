#pragma once

#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "vision/IVisionInput.h"
#include "vision/VisionInputPhotonVision.h"
#include "vision/VisionInputPhotonVisionSim.h"

struct CameraInfo {
  // The name of the camera as defined in its configuration (e.g. in the PhotonVision configuration page)
  std::string camera_name;

  // Represents the distance from the center of the robot to the camera
  frc::Transform3d robot_to_camera;

  CameraInfo() = delete;
  CameraInfo(std::string cameraName, frc::Transform3d robotToCamera)
    : camera_name(cameraName), robot_to_camera(robotToCamera) {}
};

class VisionConstants {
public:
  static inline const frc::AprilTagFieldLayout kAprilTagFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);

  static constexpr double kMaxAmbiguity = 0.3;
  static constexpr units::meter_t kMaxZError = 0.75_m;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  static constexpr units::meter_t kLinearStdDevBaseline = 0.02_m;
  static constexpr units::radian_t kAngularStdDevBaseline = 0.06_rad;

public:
  static std::vector<std::unique_ptr<IVisionInput>> GetLocalizationCameras(subsystems::CommandSwerveDrivetrain* drivetrain) {
    if (frc::RobotBase::IsSimulation()) {
      return VisionConstants::CreateSimLocalizationCameras(drivetrain);
    } else {
      return VisionConstants::CreateLocalizationCameras();
    }
  }

private:
  /**
   * Generate a list of cameras used for localization
   */
  static std::vector<std::unique_ptr<IVisionInput>> CreateLocalizationCameras() {
    std::vector<std::unique_ptr<IVisionInput>> localization_cameras;
    localization_cameras.push_back(std::make_unique<VisionInputPhotonVision>(
      "Back",
      frc::Transform3d{-22.5_in, -8.5_in, 17.5_in, frc::Rotation3d{0.0_rad, 20_deg, 180_deg}}
    ));
    localization_cameras.push_back(std::make_unique<VisionInputPhotonVision>(
      "Left",
      frc::Transform3d{-1.375_in, 32_in, 17.625_in, frc::Rotation3d{0.0_rad, 20_deg, 90_deg}}
    ));
    localization_cameras.push_back(std::make_unique<VisionInputPhotonVision>(
      "Right",
      frc::Transform3d{-1.5_in, -32_in, 17.75_in, frc::Rotation3d{0.0_rad, 20_deg, -90_deg}}
    ));

    return localization_cameras;
  }

  /**
   * Generate a list of simulated cameras for running on the simluator
   */
  static std::vector<std::unique_ptr<IVisionInput>> CreateSimLocalizationCameras(subsystems::CommandSwerveDrivetrain* drivetrain) {
    const CameraInfo kCamera0{"localization0", {0.2_m, 0.0_m, 0.2_m, frc::Rotation3d{0.0_rad, -0.4_rad, 0.0_rad}}};

    std::vector<std::unique_ptr<IVisionInput>> localization_cameras;

    localization_cameras.push_back(std::make_unique<VisionInputPhotonVisionSim>(kCamera0.camera_name, kCamera0.robot_to_camera, [drivetrain] {
      return drivetrain->GetState().Pose;
    }));

    return localization_cameras;
  }
};