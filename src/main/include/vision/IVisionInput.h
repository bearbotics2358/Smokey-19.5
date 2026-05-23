#pragma once

#include <set>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>

class IVisionInput {
public:

  enum class PoseObservationType {
    kMegaTag1 = 1,
    kMegaTag2 = 2,
    kPhotonVision = 3,
  };

  // Represents a robot pose sample used for pose estimation
  struct PoseObservation {
    units::second_t timestamp;
    frc::Pose3d pose;
    double ambiguity;
    size_t tag_count;
    units::meter_t average_tag_distance;
    PoseObservationType type;
  };

  struct Inputs {
    std::vector<PoseObservation> pose_observations;
    std::set<int> tag_ids;
  };

  virtual IVisionInput::Inputs Update() = 0;
};