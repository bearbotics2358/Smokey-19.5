#include "vision/VisionInputPhotonVisionSim.h"
#include "vision/VisionConstants.h"

VisionInputPhotonVisionSim::VisionInputPhotonVisionSim(std::string name,
                                                       frc::Transform3d robotToCamera,
                                                       std::function<frc::Pose2d()> getPose)
  : VisionInputPhotonVision(name, robotToCamera),
    m_VisionSim("main"),
    m_CameraSim(&m_Camera, photon::SimCameraProperties(), VisionConstants::kAprilTagFieldLayout),
    m_GetPose(getPose) {

  m_VisionSim.AddAprilTags(VisionConstants::kAprilTagFieldLayout);
  m_VisionSim.AddCamera(&m_CameraSim, m_RobotToCamera);
}

IVisionInput::Inputs VisionInputPhotonVisionSim::Update() {
  m_VisionSim.Update(m_GetPose());

  return VisionInputPhotonVision::Update();
}