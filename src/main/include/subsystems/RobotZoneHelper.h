#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rectangle2d.h>
#include <frc/DriverStation.h>


class RobotZoneHelper {
    public:
        enum class BumpZone {
            NoBumpZone,
            InRightBumpZone,
            InLeftBumpZone
        };
        enum class TrenchZone {
            NoTrenchZone,
            InRightTrenchZone,
            InLeftTenchZone
        };
        
        static bool isRobotInBlueAllianceZone (frc::Pose2d botPose) {
            frc::Rectangle2d blueAllianceZone = frc::Rectangle2d(
                frc::Pose2d(91.05_in, 158.84_in, 
                frc::Rotation2d()), 
                182.11_in, 317.69_in
            );

            return blueAllianceZone.Contains(botPose.Translation());
        }

        static bool isRobotInRedAllianceZone (frc::Pose2d botPose) {
            frc::Rectangle2d redAllianceZone = frc::Rectangle2d(
                frc::Pose2d(560.17_in, 158.84_in, 
                frc::Rotation2d()), 
                182.11_in, 317.69_in
            );

            return redAllianceZone.Contains(botPose.Translation());
        }

        static bool isRobotInNeutralZone (frc::Pose2d botPose) {
            frc::Rectangle2d neutralZone = frc::Rectangle2d(
                frc::Pose2d(325.61_in, 158.84_in, 
                frc::Rotation2d()), 
                287_in, 317.69_in
            );

            return neutralZone.Contains(botPose.Translation());
        }

        static bool isRobotInMyAllianceZone(frc::Pose2d botPose) {
            frc::DriverStation::Alliance currentAlliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);

            if (currentAlliance == frc::DriverStation::Alliance::kBlue) {
                return isRobotInBlueAllianceZone(botPose);
            } else {
                return isRobotInRedAllianceZone(botPose);
            }
        }

        static TrenchZone isRobotInTrenchZone(frc::Pose2d botPose) {
            frc::Rectangle2d blueRightTrench = frc::Rectangle2d(
                frc::Pose2d(182.11_in, 26.22_in, 
                frc::Rotation2d()), 
                300_in, 69.84_in
            );
            frc::Rectangle2d blueLeftTrench = frc::Rectangle2d(
                frc::Pose2d(182.11_in, 291.47_in, 
                frc::Rotation2d()), 
                300_in, 69.84_in
            );
            frc::Rectangle2d redRightTrench = frc::Rectangle2d(
                frc::Pose2d(469.11_in, 26.22_in, 
                frc::Rotation2d()), 
                300_in, 69.84_in
            );
            frc::Rectangle2d redLeftTrench = frc::Rectangle2d(
                frc::Pose2d(469.11_in, 291.47_in, 
                frc::Rotation2d()), 
                300_in, 69.84_in
            );

            if (blueRightTrench.Contains(botPose.Translation()) || redRightTrench.Contains(botPose.Translation())) {
                return TrenchZone::InRightTrenchZone;
            } else if (blueLeftTrench.Contains(botPose.Translation()) || redLeftTrench.Contains(botPose.Translation())) {
                return TrenchZone::InLeftTenchZone;
            } else {
                return TrenchZone::NoTrenchZone;
            }
        }

        static BumpZone isRobotInBumpZone(frc::Pose2d botPose) {
            frc::Rectangle2d blueRightBump = frc::Rectangle2d(
                frc::Pose2d(182.11_in, 111.84_in, 
                frc::Rotation2d()), 
                200_in, 1.854_m
            );
            frc::Rectangle2d blueLeftBump = frc::Rectangle2d(
                frc::Pose2d(182.11_in, 205.85_in, 
                frc::Rotation2d()), 
                200_in, 1.854_m
            );
            frc::Rectangle2d redRightBump = frc::Rectangle2d(
                frc::Pose2d(469.11_in, 111.84_in, 
                frc::Rotation2d()), 
                200_in, 1.854_m
            );
            frc::Rectangle2d redLeftBump = frc::Rectangle2d(
                frc::Pose2d(469.11_in, 205.85_in, 
                frc::Rotation2d()), 
                200_in, 1.854_m
            );

            if (blueRightBump.Contains(botPose.Translation()) || redRightBump.Contains(botPose.Translation())) {
                return BumpZone::InRightBumpZone;
            } else if (blueLeftBump.Contains(botPose.Translation()) || redLeftBump.Contains(botPose.Translation())) {
                return BumpZone::InLeftBumpZone;
            } else {
                return BumpZone::NoBumpZone;
            }
        }
};