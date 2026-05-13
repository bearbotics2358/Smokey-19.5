#pragma once

#include <functional>

#include "trajectory/TrajectoryCalc.h"
#include <frc/kinematics/ChassisSpeeds.h>

#include <units/acceleration.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

class LaunchHelper {
public:
    static LaunchHelper& GetInstance();
    void Init(std::function<units::degree_t()> hoodAngleSupplier,
              std::function<units::revolutions_per_minute_t()> shooterSpeedSupplier,
              std::function<units::degree_t()> turretAngleSupplier,
              std::function<frc::ChassisSpeeds()> robotSpeedsSupplier,
              std::function<frc::Pose2d()> robotPoseSupplier);

    TrajectoryInfo GetLaunchParameters();

    // Delete the copy constructor. LaunchHelper should not be cloneable since it is a singleton.
    LaunchHelper(const LaunchHelper&) = delete;

    // This is a singleton class so prevent assigning a LaunchHelper object
    LaunchHelper& operator=(const LaunchHelper&) = delete;


    void DrawTrajectory();
private:
    LaunchHelper();

    units::meters_per_second_t RPMToVelocity(units::revolutions_per_minute_t rpm);

    bool m_Initialized = false;
    TrajectoryCalc m_TrajectoryCalc;

    static constexpr units::meter_t kFlywheelRadius = 0.05_m;

    // Store the TrajectoryInfo so that rapid requests from various subsystems don't all run compute_trajectory
    TrajectoryInfo m_Cache;

    // This is how long the m_Cache values are good for
    static constexpr units::millisecond_t kCacheTimeout = 20_ms;
    units::second_t m_LastCacheTime = 0_s;

    std::function<units::degree_t()> m_GetHoodAngle;
    std::function<units::revolutions_per_minute_t()> m_GetShooterSpeed;
    std::function<units::degree_t()> m_GetTurretAngle;
    std::function<frc::ChassisSpeeds()> m_RobotSpeedsSupplier;
    std::function<frc::Pose2d()> m_RobotPoseSupplier;
};