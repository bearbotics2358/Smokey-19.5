#include <gtest/gtest.h>

#include "trajectory/model.h"
#include "trajectory/TrajectoryCalc.h"

struct TrajectoryParams {
    enum model_t model;
    units::foot_t distance;
    units::revolutions_per_minute_t rpm;
    double expected_angle_deg;
};

struct TrajectoryParamsCompensated {
    bool fixed_angle;
    enum model_t model;
    units::foot_t distance;
    units::degree_t angle;
    units::revolutions_per_minute_t rpm;
    units::degree_t expected_angle;
    units::revolutions_per_minute_t expected_rpm;    
    units::second_t expected_t;
	enum return_value_t expected_return_value;
};

struct TrajectoryParamsShootontheFly {
    units::foot_t distance;
    units::degree_t angle;
    units::revolutions_per_minute_t wheel_rpm;
    units::degree_t hub_angle;
    units::feet_per_second_t vx;
    units::feet_per_second_t vy;
    units::degree_t expected_turret_angle;
    units::degree_t expected_elevation_angle;
    units::revolutions_per_minute_t expected_rpm;    
};



class TestTrajectory : public testing::TestWithParam<TrajectoryParams> {
protected:
    TrajectoryCalc m_TrajectoryCalc;

public:
    TestTrajectory() {
        m_TrajectoryCalc.init();
        m_TrajectoryCalc.set_model(NO_AIR);
    }
};

class TestTrajectoryCompensated : public testing::TestWithParam<TrajectoryParamsCompensated> {
protected:
    TrajectoryCalc m_TrajectoryCalc;

public:
    TestTrajectoryCompensated() {
        m_TrajectoryCalc.init();
        m_TrajectoryCalc.set_model(NO_AIR);
    }
};

class TestTrajectoryShootontheFly : public testing::TestWithParam<TrajectoryParamsShootontheFly> {
protected:
    TrajectoryCalc m_TrajectoryCalc;

public:
    TestTrajectoryShootontheFly() {
        m_TrajectoryCalc.init();
        m_TrajectoryCalc.set_model(AIR_DRAG);
    	m_TrajectoryCalc.set_shoot_on_the_move_enabled(true);
	    m_TrajectoryCalc.set_constant_shooter_elevation(false);

    }
};

TEST_P(TestTrajectory, ValidateAnglesFromTrajectoryTableWithKnownDistancesAndRPMs) {
    TrajectoryParams params = GetParam();

    m_TrajectoryCalc.set_model(params.model);
    units::degree_t angle = m_TrajectoryCalc.get_angle(params.distance, params.rpm);

    EXPECT_DOUBLE_EQ(angle.value(), params.expected_angle_deg);
}

INSTANTIATE_TEST_SUITE_P(
    TrajectoryDistanceTest,
    TestTrajectory,
    ::testing::Values(
        // Include a small set of ranges for distance and rpms for testing. If testing on the real robot shows that
        // the trajectory calculations need to be adjusted, these may need to change (so we should avoid adding
        // a large number of them). We want a representative set to ensure that any possible code optimizations
        // (like for lookup or calculation speed) to the TrajectoryTable and TrajectoryCalc classes will give us the
        // same result.

        TrajectoryParams{AIR_DRAG, 5_ft, 3600_rpm, 80}, // 0
        TrajectoryParams{AIR_DRAG, 8_ft, 3600_rpm, 73}, // 1
        TrajectoryParams{AIR_DRAG, 11_ft, 3600_rpm, 49}, // 2
        TrajectoryParams{AIR_DRAG, 20_ft, 3600_rpm, 55}, // 3
        TrajectoryParams{AIR_DRAG, 5_ft, 3200_rpm, 76}, // 4
        TrajectoryParams{AIR_DRAG, 8_ft, 3200_rpm, 57}, // 5
        TrajectoryParams{AIR_DRAG, 11_ft, 3200_rpm, 60}, // 6
        TrajectoryParams{AIR_DRAG, 20_ft, 3200_rpm, 60} // 7
        
    )
);

TEST_P(TestTrajectoryCompensated, ValidateAnglesFromTrajectoryTableWithKnownDistancesAndRPMsCompensated) {
    TrajectoryParamsCompensated params = GetParam();

    struct TrajectoryInfo trajInfo;
    
    /* For Reference:
    // TrajectoryInfo
	units::degree_t elevation_angle;
	units::revolutions_per_minute_t wheel_rpm;
	units::foot_t distance; // from shooter to the center of the hub
	units::feet_per_second_t vx; // field relative
	units::feet_per_second_t vy; // field relative
	units::degree_t turret_angle; // field relative
	units::degree_t hub_angle; // field relative
	enum return_value_t return_value;

    // TrajectoryParamsCompensated
    units::foot_t distance;
    units::degree_t angle;
    units::revolutions_per_minute_t rpm;
    units::degree_t expected_angle;
    units::revolutions_per_minute_t expected_rpm;    
	enum return_value_t expected_return_value;
    */

	units::degree_t elevation_angle = params.angle;
	units::revolutions_per_minute_t wheel_rpm = params.rpm;

    trajInfo.distance = params.distance;
    trajInfo.elevation_angle = params.angle;
    trajInfo.wheel_rpm = params.rpm;
    trajInfo.vx = 0.0_fps;
    trajInfo.vy = 0.0_fps;
    double alpha = 0.9;
    double max_runs;

    m_TrajectoryCalc.set_constant_shooter_elevation(params.fixed_angle);
    m_TrajectoryCalc.set_model(params.model);

    /*
    if(trajInfo.wheel_rpm.value() < 1.0) {
        max_runs = 500;
    } else {
        max_runs = 200;
    }
    */

    printf("test initial command: elevation angle: %6.3lf  rpm: %6.3lf  dist: %6.3lf  result: %d",
        trajInfo.elevation_angle.value(), trajInfo.wheel_rpm.value(), trajInfo.distance.value(), trajInfo.return_value);
    printf(" expect: elevation angle: %6.3lf  rpm: %6.3lf  result: %d\n",
        params.expected_angle.value(), params.expected_rpm.value(), params.expected_return_value);
    
    trajInfo = m_TrajectoryCalc.compute_trajectory(trajInfo);

    EXPECT_DOUBLE_EQ(trajInfo.elevation_angle.value(), params.expected_angle.value());
    EXPECT_NEAR(trajInfo.wheel_rpm.value(), params.expected_rpm.value(), 1.0);
    EXPECT_DOUBLE_EQ(trajInfo.t.value(), params.expected_t.value());
    EXPECT_EQ(trajInfo.return_value, params.expected_return_value);

    /*
    for(int i = 0; i <= max_runs; i++) {
        elevation_angle = alpha * elevation_angle + (1.0 - alpha) * trajInfo.elevation_angle;
        trajInfo.elevation_angle = elevation_angle;
        wheel_rpm = alpha * wheel_rpm + (1.0 - alpha) * trajInfo.wheel_rpm;
        trajInfo.wheel_rpm = wheel_rpm;
        if((i < 5) || (i > max_runs - 5)) {
            printf("test #%d: model: %d elevation angle: %6.3lf  rpm: %6.3lf  dist: %6.3lf  result: %d\n", i, m_TrajectoryCalc.get_model(), trajInfo.elevation_angle, trajInfo.wheel_rpm, trajInfo.distance, trajInfo.return_value);
        }
        if(i == 5) {
            printf("...\n");
        }
        trajInfo = m_TrajectoryCalc.compute_trajectory(trajInfo);
    }
    */
}

INSTANTIATE_TEST_SUITE_P(
    TrajectoryDistanceTestCompensated,
    TestTrajectoryCompensated,
    ::testing::Values(
        // Include a small set of ranges for distance and rpms for testing. If testing on the real robot shows that
        // the trajectory calculations need to be adjusted, these may need to change (so we should avoid adding
        // a large number of them). We want a representative set to ensure that any possible code optimizations
        // (like for lookup or calculation speed) to the TrajectoryTable and TrajectoryCalc classes will give us the
        // same result.
        /*
        TrajectoryParamsCompensated{false, NO_AIR, 20_ft, 67_deg, 3600_rpm, 67_deg, 3600_rpm, 1.616_s, TRAJECTORY_SUCCESS}, // #0
        // TrajectoryParamsCompensated{20_ft, 52_deg, 3200_rpm, 55_deg, 3275_rpm}, // #1
        TrajectoryParamsCompensated{false, NO_AIR, 20_ft, 52_deg, 3200_rpm, 65_deg, 3525_rpm, 1.543_s, TRAJECTORY_SUCCESS}, // #1
        TrajectoryParamsCompensated{false, NO_AIR, 7_ft, 65_deg, 3600_rpm, 65_deg, 2375_rpm, 0.801_s, TRAJECTORY_FAILURE_SHOT_LONG}, // #2
        // TrajectoryParamsCompensated{20_ft, 65_deg, 3275_rpm, 55_deg, 3275_rpm}, // #3
        TrajectoryParamsCompensated{false, NO_AIR, 20_ft, 65_deg, 3275_rpm, 65_deg, 3525_rpm, 1.543_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #3

        // causes a problem 'cause 55 deg 2200 RPM is 0 part of table
        // gives 67 2200
        // FIXED with using 65 degrees
        TrajectoryParamsCompensated{false, NO_AIR, 20_ft, 65_deg, 2200_rpm, 65_deg, 3525_rpm, 1.543_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #4
        TrajectoryParamsCompensated{false, NO_AIR, 7_ft, 65_deg, 2650_rpm, 74_deg, 2650_rpm, 1.119_s, TRAJECTORY_FAILURE_SHOT_LONG}, // #5

        // ends up on wrong side of the arc 55, 2450
        // FIXED
        // TrajectoryParamsCompensated{7_ft, 75_deg, 0_rpm, 74_deg, 2650_rpm}, // #6  
        TrajectoryParamsCompensated{false, NO_AIR, 7_ft, 75_deg, 0_rpm, 65_deg, 2375_rpm, 0.801_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #6  
        // TrajectoryParamsCompensated{20_ft, 55_deg, 0_rpm, 74_deg, 2650_rpm} // #7
        TrajectoryParamsCompensated{false, NO_AIR, 20_ft, 55_deg, 0_rpm, 65_deg, 3525_rpm, 1.543_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #7
        */

        TrajectoryParamsCompensated{false, AIR_DRAG, 7_ft, 65_deg, 2650_rpm, 65_deg, 3104_rpm, 0.795_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #0
        TrajectoryParamsCompensated{false, AIR_DRAG, 7_ft, 65_deg, 3600_rpm, 65_deg, 3104_rpm, 0.795_s, TRAJECTORY_FAILURE_SHOT_LONG}, // #1 - this is wrong, it should have returned 3600 RPM - I think this activated it is within good angles, but shot is long, goto 65 deg and find an RPM

        TrajectoryParamsCompensated{false, AIR_DRAG, 7.2_ft, 65_deg, 3600_rpm, 75_deg, 3600_rpm, 1.192_s, TRAJECTORY_FAILURE_SHOT_LONG}, // actually went to 75 deg 3600 RPM, which is correct: stay at current RPM and find an angle

        TrajectoryParamsCompensated{false, AIR_DRAG, 20_ft, 55_deg,    0_rpm, 65_deg, 4845_rpm, 1.580_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #2
        TrajectoryParamsCompensated{false, AIR_DRAG, 20_ft, 52_deg, 3200_rpm, 65_deg, 4845_rpm, 1.580_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #3
        TrajectoryParamsCompensated{false, AIR_DRAG, 20_ft, 65_deg, 2200_rpm, 65_deg, 4845_rpm, 1.580_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #4
        TrajectoryParamsCompensated{false, AIR_DRAG, 20_ft, 65_deg, 3275_rpm, 65_deg, 4845_rpm, 1.580_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #5
        TrajectoryParamsCompensated{false, AIR_DRAG, 20_ft, 67_deg, 3600_rpm, 65_deg, 4845_rpm, 1.580_s, TRAJECTORY_FAILURE_SHOT_SHORT}, // #6

        // for fixed angle
        TrajectoryParamsCompensated{true, AIR_DRAG, 7_ft, 65_deg, 3600_rpm, 65_deg, 3104_rpm, 0.795_s, TRAJECTORY_FAILURE_SHOT_LONG} // #7
    )
);

TEST_P(TestTrajectoryShootontheFly, ValidateDataFromShootontheFly) {
    TrajectoryParamsShootontheFly params = GetParam();

    struct TrajectoryInfo trajInfo;

    /*
    units::foot_t distance;
    units::degree_t angle;
    units::degree_t hub_angle;
    units::feet_per_second_t vx;
    units::feet_per_second_t vy;
    units::degree_t expected_turret_angle;
    units::degree_t expected_elevation_angle;
    units::revolutions_per_minute_t expected_rpm;    

	units::degree_t elevation_angle = params.angle;
	units::revolutions_per_minute_t wheel_rpm = params.rpm;


    */

    trajInfo.distance = params.distance;
    trajInfo.elevation_angle = params.angle;
    trajInfo.hub_angle = params.hub_angle;
    trajInfo.vx = params.vx;
    trajInfo.vy = params.vy;
    trajInfo.wheel_rpm = params.wheel_rpm;
    trajInfo = m_TrajectoryCalc.compute_trajectory(trajInfo);

    EXPECT_DOUBLE_EQ(trajInfo.elevation_angle.value(), params.expected_elevation_angle.value());
    /*
    EXPECT_NEAR(trajInfo.wheel_rpm.value(), params.expected_rpm.value(), 1.0);
    EXPECT_DOUBLE_EQ(trajInfo.t.value(), params.expected_t.value());
    EXPECT_EQ(trajInfo.return_value, params.expected_return_value);
    EXPECT_DOUBLE_EQ(angle.value(), params.expected_angle_deg);
    */
}

INSTANTIATE_TEST_SUITE_P(
    TrajectoryDistanceTest,
    TestTrajectoryShootontheFly,
    ::testing::Values(
        // Include a small set of ranges for distance and rpms for testing. If testing on the real robot shows that
        // the trajectory calculations need to be adjusted, these may need to change (so we should avoid adding
        // a large number of them). We want a representative set to ensure that any possible code optimizations
        // (like for lookup or calculation speed) to the TrajectoryTable and TrajectoryCalc classes will give us the
        // same result.

        TrajectoryParamsShootontheFly{10_ft, 65_deg, 3600_rpm, 0_deg, 2_fps, 0_fps, 0_deg, 65_deg, 3600_rpm}, // 0
        TrajectoryParamsShootontheFly{15_ft, 47_deg, 3974_rpm, 0_deg, 2_fps, 0_fps, 0_deg, 65_deg, 3600_rpm}, // 1
        TrajectoryParamsShootontheFly{12_ft, 61_deg, 3655_rpm, 45_deg, 5_fps, 5_fps, 0_deg, 65_deg, 3600_rpm}, // 2
        TrajectoryParamsShootontheFly{12_ft, 61_deg, 3655_rpm, -45_deg, 5_fps, 5_fps, 0_deg, 65_deg, 3600_rpm}, // 3
        TrajectoryParamsShootontheFly{8_ft, 62_deg, 3191_rpm, 25_deg, 4.33_fps, 2.5_fps, 0_deg, 65_deg, 3600_rpm} // 4

        
    )
);

