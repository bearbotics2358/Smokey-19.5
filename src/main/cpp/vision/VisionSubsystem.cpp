#include "frc/RobotBase.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include <units/math.h>

#include "bearlog/bearlog.h"
#include "vision/VisionConstants.h"
#include "vision/VisionSubsystem.h"
#include "vision/VisionInputPhotonVisionSim.h"

VisionSubsystem::VisionSubsystem(subsystems::CommandSwerveDrivetrain* drivetrain,
                                 std::vector<std::unique_ptr<IVisionInput>> cameras)
    : m_Drivetrain(drivetrain), m_LocalizationCameras(std::move(cameras))
{
  // Initialize the vector of inputs to be the size of the number of cameras
  for (size_t ii = 0; ii < m_LocalizationCameras.size(); ii++) {
    m_Inputs.push_back(IVisionInput::Inputs{});
  }
}

void VisionSubsystem::Periodic()
{
  for (size_t ii = 0; ii < m_LocalizationCameras.size(); ii++) {
    m_Inputs[ii] = m_LocalizationCameras.at(ii)->Update();
  }

  for (size_t camera_index = 0; camera_index < m_LocalizationCameras.size(); camera_index++) {
    std::vector<frc::Pose3d> tag_poses;
    std::vector<frc::Pose3d> robot_poses_accepted;
    std::vector<frc::Pose3d> robot_poses_rejected;

    for (int tag_id : m_Inputs[camera_index].tag_ids) {
      std::optional<frc::Pose3d> tag_pose = VisionConstants::kAprilTagFieldLayout.GetTagPose(tag_id);
      if (tag_pose) {
        tag_poses.push_back(tag_pose.value());
      }
    }

    for (auto observation : m_Inputs[camera_index].pose_observations) {
      bool reject_pose =
        observation.tag_count == 0
        || (observation.tag_count == 1 && observation.ambiguity > VisionConstants::kMaxAmbiguity)
        || units::math::abs(observation.pose.Z()) > VisionConstants::kMaxZError
        || observation.pose.X() < 0_m
        || observation.pose.X() > VisionConstants::kAprilTagFieldLayout.GetFieldLength()
        || observation.pose.Y() < 0_m
        || observation.pose.Y() > VisionConstants::kAprilTagFieldLayout.GetFieldWidth();

      if (reject_pose) {
        robot_poses_rejected.push_back(observation.pose);

        // Skip to the next observation if this one is rejected
        continue;
      } else {
        robot_poses_accepted.push_back(observation.pose);
      }

      auto std_dev_factor = units::math::pow<2, units::meter_t>(observation.average_tag_distance) / observation.tag_count;
      auto linear_std_dev = VisionConstants::kLinearStdDevBaseline * std_dev_factor;
      auto angular_std_dev = VisionConstants::kAngularStdDevBaseline * std_dev_factor;

      m_Drivetrain->AddVisionMeasurement(observation.pose.ToPose2d(),
                                         observation.timestamp);//,
                                         // @todo Include the standard deviation calculations here when we are confident in them
                                        //  {linear_std_dev.value(), linear_std_dev.value(), angular_std_dev()});
    }

    BearLog::Log("Vision/Camera" + std::to_string(camera_index) + "/TagPoses", tag_poses);
    BearLog::Log("Vision/Camera" + std::to_string(camera_index) + "/RobotPosesAccepted", robot_poses_accepted);
    BearLog::Log("Vision/Camera" + std::to_string(camera_index) + "/RobotPosesRejected", robot_poses_rejected);
  }

  BearLog::Log("Vision/PigeonYaw", m_Drivetrain->GetPigeon2().GetYaw().GetValue());
}