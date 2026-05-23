#include "vision/VisionInputPhotonVision.h"
#include "vision/VisionConstants.h"

VisionInputPhotonVision::VisionInputPhotonVision(std::string name, frc::Transform3d robotToCamera)
    : m_Camera(name),
      m_RobotToCamera(robotToCamera)
{
}

IVisionInput::Inputs VisionInputPhotonVision::Update()
{
  // Clear out all previously seen tag IDs to add the ones seen in this iteration of the loop
  m_CameraInputs.tag_ids.clear();

  // Clear out all previous pose observations to add the ones seen in this iteration of the loop
  m_CameraInputs.pose_observations.clear();

  for (auto result : m_Camera.GetAllUnreadResults()) {
    // Add pose observation
    if (result.MultiTagResult()) {
      auto multi_tag_result = result.MultiTagResult().value();

      // Calculate robot pose
      frc::Transform3d field_to_camera = multi_tag_result.estimatedPose.best;
      frc::Transform3d field_to_robot = field_to_camera + m_RobotToCamera.Inverse();
      frc::Pose3d robot_pose{field_to_robot.Translation(), field_to_robot.Rotation()};

      // Calculate average tag distance
      units::meter_t total_tag_distance = 0_m;
      for (auto target : result.targets) {
        total_tag_distance += target.GetBestCameraToTarget().Translation().Norm();
      }

      // Update the set of tag IDs with all of the ones from the result
      m_CameraInputs.tag_ids.insert(multi_tag_result.fiducialIDsUsed.cbegin(), multi_tag_result.fiducialIDsUsed.cend());

      PoseObservation new_pose_observation{
        result.GetTimestamp(),
        robot_pose,
        multi_tag_result.estimatedPose.ambiguity,
        multi_tag_result.fiducialIDsUsed.size(),
        total_tag_distance / result.targets.size(),
        PoseObservationType::kPhotonVision
      };

      m_CameraInputs.pose_observations.push_back(new_pose_observation);
    } else if (result.targets.size() > 0) {
      // Single tag result

      auto target = result.targets.at(0);

      // Calculate robot pose
      std::optional<frc::Pose3d> tag_pose = VisionConstants::kAprilTagFieldLayout.GetTagPose(target.GetFiducialId());
      if (tag_pose) {
        frc::Transform3d field_to_target{tag_pose->Translation(), tag_pose->Rotation()};
        frc::Transform3d camera_to_target{target.GetBestCameraToTarget()};
        frc::Transform3d field_to_camera = field_to_target + camera_to_target.Inverse();
        frc::Transform3d field_to_robot = field_to_camera + m_RobotToCamera.Inverse();
        frc::Pose3d robot_pose{field_to_robot.Translation(), field_to_robot.Rotation()};

        // Update the set of tag IDs with the one from the result
        m_CameraInputs.tag_ids.insert(target.GetFiducialId());

        PoseObservation new_pose_observation{
          result.GetTimestamp(),
          robot_pose,
          target.GetPoseAmbiguity(),
          1, // Only 1 tag used in this observation
          camera_to_target.Translation().Norm(),
          PoseObservationType::kPhotonVision
        };

        m_CameraInputs.pose_observations.push_back(new_pose_observation);
      }
    }
  }

  return m_CameraInputs;
}