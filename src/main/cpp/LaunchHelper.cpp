#include <iostream>

#if defined(_WIN32) || defined(_WIN64)
#define _USE_MATH_DEFINES
#endif

#include "FieldConstants.h"
#include "LaunchHelper.h"
#include "bearlog/bearlog.h"
#include <frc/Timer.h>
#include <frc/RobotBase.h>

LaunchHelper& LaunchHelper::GetInstance() {
    // Defining a static LaunchHelper object so it will only be created once
    static LaunchHelper instance;
    return instance;
}

LaunchHelper::LaunchHelper()
    : m_LastCacheTime{frc::Timer::GetFPGATimestamp()}
{
}

void LaunchHelper::Init(std::function<units::degree_t()> hoodAngleSupplier,
                        std::function<units::revolutions_per_minute_t()> shooterSpeedSupplier,
                        std::function<units::degree_t()> turretAngleSupplier,
                        std::function<frc::ChassisSpeeds()> robotSpeedsSupplier,
                        std::function<frc::Pose2d()> robotPoseSupplier) {
    m_GetHoodAngle = hoodAngleSupplier;
    m_GetShooterSpeed = shooterSpeedSupplier;
    m_GetTurretAngle = turretAngleSupplier;
    m_RobotSpeedsSupplier = robotSpeedsSupplier;
    m_RobotPoseSupplier = robotPoseSupplier;

    m_TrajectoryCalc.init();
    m_TrajectoryCalc.set_constant_shooter_elevation(true);

    m_TrajectoryCalc.set_ball_compression(0.422);

    m_Initialized = true;
}

TrajectoryInfo LaunchHelper::GetLaunchParameters() {
    TrajectoryInfo inputs;

    if (false == m_Initialized) {
        std::cout << "[ERROR] LaunchHelper::GetLaunchParameters called while uninitialized. Did you forget to call Init?" << std::endl;
        return inputs;
    }

    units::second_t timestamp = frc::Timer::GetFPGATimestamp();
    if (timestamp - m_LastCacheTime < kCacheTimeout) {
        // Return the most recently calculated values until the cache is considered stale (after kCacheTimeout)
        return m_Cache;
    }

    m_LastCacheTime = timestamp;

    inputs.elevation_angle = m_GetHoodAngle();
    inputs.wheel_rpm = m_GetShooterSpeed();
    inputs.turret_angle = m_GetTurretAngle();

    frc::Translation2d hub_center = FieldConstants::GetHubCenterForMyAlliance();
    units::meter_t distance_to_hub_center = units::math::abs(m_RobotPoseSupplier().Translation().Distance(hub_center));
    inputs.distance = distance_to_hub_center;

    BearLog::Log("LaunchHelper/Distance to Hub", units::foot_t(distance_to_hub_center));

    // @todo When we're ready for shoot on the move, these should be
    //  updated to use m_RobotSpeedsSupplier().vx and m_RobotSpeedsSupplier().vy
    inputs.vx = 0_fps;
    inputs.vy = 0_fps;

    m_Cache = m_TrajectoryCalc.compute_trajectory(inputs);

    // m_Cache.wheel_rpm += (1500_rpm * units::foot_t(distance_to_hub_center).value

    BearLog::Log("LaunchHelper/Hood Angle", m_Cache.elevation_angle);
    BearLog::Log("LaunchHelper/Wheel RPM", m_Cache.wheel_rpm);
    BearLog::Log("LaunchHelper/Turret Angle", m_Cache.turret_angle);

    if (frc::RobotBase::IsSimulation()) {
        DrawTrajectory();
    }
    return m_Cache;
}

void LaunchHelper::DrawTrajectory()
{
        units::meters_per_second_t velocity = RPMToVelocity(m_GetShooterSpeed());
        units::degree_t angle = m_GetHoodAngle();
        units::meters_per_second_squared_t gravity = 9.81_mps_sq;
        std::vector<frc::Pose3d> poses;
        double timeBetweenPoses = 0.05;

        units::meter_t shooterHeight = 18.2817_in;
        frc::Pose2d botPose2d = m_RobotPoseSupplier();
        frc::Pose3d robotPose3d{
            botPose2d.X(),
            botPose2d.Y(),
            0_m,
            frc::Rotation3d{0_deg, 0_deg, botPose2d.Rotation().Degrees()}
        };

        units::radian_t degRad = units::radian_t(angle);

        BearLog::Log("velocity", velocity);

        // units::second_t timeOfFlight = units::second_t(((velocity.value() * sin(degRad.value())) +
        //     std::sqrt(std::pow(velocity.value() * sin(degRad.value()), 2) +
        //         (2.0 * gravity.value() * shooterHeight.value())
        //     )
        // ) / gravity.value());

        double hDist = 0;
        double vDist = shooterHeight.value();

        double airDensity = 1.225;
        double dragCoefficient = 0.47; //0.47
        double crossSectionArea = M_PI * pow((0.150114 / 2.0), 2);
        double mass = 0.227;

        double dragConstant = 0.5 * airDensity * dragCoefficient * crossSectionArea;

        double horizontalVelocity = velocity.value() * cos(degRad.value());
        double verticalVelocity = velocity.value() * sin(degRad.value());
        while ((vDist >= 0)) {
            // hDist = velocity.value() * cos(degRad.value()) * time;
            // vDist = shooterHeight.value() + (velocity.value() * sin(degRad.value()) * time) - (0.5 * gravity.value() * pow(time, 2));

            double totalSpeed = sqrt(horizontalVelocity * horizontalVelocity + verticalVelocity * verticalVelocity);

            double horizontalDrag = -(dragConstant * totalSpeed * horizontalVelocity) / mass;
            double verticalDrag = -gravity.value() - (dragConstant * totalSpeed * verticalVelocity) / mass;

            horizontalVelocity += horizontalDrag * timeBetweenPoses;
            verticalVelocity += verticalDrag * timeBetweenPoses;

            hDist += horizontalVelocity * timeBetweenPoses;
            vDist += verticalVelocity * timeBetweenPoses;

            frc::Translation3d localPose{
                units::meter_t(hDist),
                0_m,
                units::meter_t(vDist)
            };

            frc::Translation3d rotated = localPose.RotateBy(
            frc::Rotation3d{0_deg, 0_deg, botPose2d.Rotation().Degrees() + m_GetTurretAngle()});

            frc::Pose3d worldPose{
                robotPose3d.X() + rotated.X(),
                robotPose3d.Y() + rotated.Y(),
                robotPose3d.Z() + rotated.Z(),
                frc::Rotation3d{}
            };

            poses.push_back(worldPose);
        }

        BearLog::Log("trajectory/trajectory", poses);
}


units::meters_per_second_t LaunchHelper::RPMToVelocity(units::revolutions_per_minute_t rpm) {
    units::turns_per_second_t revPerSec = rpm;
    units::meter_t flywheelCircumfrence = 2 * M_PI * kFlywheelRadius;
    return units::meters_per_second_t(revPerSec.value() * flywheelCircumfrence.value() / 2);
}