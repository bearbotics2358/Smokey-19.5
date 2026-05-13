#pragma once

#include <functional>

#include "vision/IVisionInput.h"
#include "vision/VisionInputPhotonVision.h"

#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/SimCameraProperties.h>
#include <photon/simulation/VisionSystemSim.h>

class VisionInputPhotonVisionSim : public VisionInputPhotonVision {
public:
  VisionInputPhotonVisionSim(std::string name,
                             frc::Transform3d robotToCamera,
                             std::function<frc::Pose2d()> getPose);

  IVisionInput::Inputs Update() override;

private:
  photon::VisionSystemSim m_VisionSim;
  photon::PhotonCameraSim m_CameraSim;
  std::function<frc::Pose2d()> m_GetPose;
};