#pragma once

#include <set>

#include "vision/IVisionInput.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>
#include <photon/PhotonCamera.h>

class VisionInputPhotonVision: public IVisionInput {
public:
  VisionInputPhotonVision(std::string name, frc::Transform3d robotToCamera);

  // From IVisionInput parent
  virtual IVisionInput::Inputs Update() override;

protected:
  photon::PhotonCamera m_Camera;
  frc::Transform3d m_RobotToCamera;

private:
  IVisionInput::Inputs m_CameraInputs;
};