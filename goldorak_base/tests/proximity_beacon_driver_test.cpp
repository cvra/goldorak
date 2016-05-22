#include "../src/UavcanRosProximityBeaconDriver.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <stdint.h>
#include <math.h>

TEST(ProximityBeaconDriverTestSuite, canComputePolarToCartesianCoordinatesTransformXPlus)
{
    float x, y;
    polar_to_cartesian(1.f, 0.f, &x, &y);
    EXPECT_NEAR(x, 1.f, 1e-7);
    EXPECT_NEAR(y, 0.f, 1e-7);
}

TEST(ProximityBeaconDriverTestSuite, canComputePolarToCartesianCoordinatesTransformXMinus)
{
    float x, y;
    polar_to_cartesian(1.f, M_PI, &x, &y);
    EXPECT_NEAR(x, -1.f, 1e-7);
    EXPECT_NEAR(y, 0.f, 1e-7);
}

TEST(ProximityBeaconDriverTestSuite, canComputePolarToCartesianCoordinatesTransformYPlus)
{
    float x, y;
    polar_to_cartesian(1.f, M_PI / 2.f, &x, &y);
    EXPECT_NEAR(x, 0.f, 1e-7);
    EXPECT_NEAR(y, 1.f, 1e-7);
}

TEST(ProximityBeaconDriverTestSuite, canComputePolarToCartesianCoordinatesTransformYMinus)
{
    float x, y;
    polar_to_cartesian(1.f, 1.5 * M_PI, &x, &y);
    EXPECT_NEAR(x, 0.f, 1e-7);
    EXPECT_NEAR(y, -1.f, 1e-7);
}

TEST(ProximityBeaconDriverTestSuite, canComputePolarToCartesianCoordinatesTransform)
{
    float x, y;
    polar_to_cartesian(1.f, M_PI / 4.f, &x, &y);
    EXPECT_NEAR(x, sqrtf(2) / 2.f, 1e-7);
    EXPECT_NEAR(y, sqrtf(2) / 2.f, 1e-7);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "proximity_beacon_driver_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
