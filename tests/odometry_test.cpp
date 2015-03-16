#include "CppUTest/TestHarness.h"
#include "../robot_base.h"
#include "../odometry.h"
#include "../odometry.c"

TEST_GROUP(Encoder)
{
    odometry_encoder_sample_t sample1;
    odometry_encoder_sample_t sample2;

    void setup(void)
    {
        encoder_record_sample(&sample1, 20000, 42);
        encoder_record_sample(&sample2, 25000, 42);
    }
};

TEST(Encoder, EncoderRecord)
{
    CHECK_EQUAL(20000, sample1.timestamp);
    CHECK_EQUAL(42, sample1.value);
}


TEST_GROUP(Wheel)
{
    odometry_wheel_t wheel;
    odometry_encoder_sample_t sample1;

    void setup(void)
    {
        wheel_init(&wheel, 0.5f);
        encoder_record_sample(&sample1, 10000, 42);
        wheel_update(&wheel, sample1);
    }
};

TEST(Wheel, WheelInit)
{
    DOUBLES_EQUAL(4.793689962e-5, wheel.tick_to_meter, 1e-11);
    CHECK_EQUAL(0, wheel.delta_pos_accumulator);

    CHECK_EQUAL(10000, wheel.samples[0].timestamp);
    CHECK_EQUAL(10000, wheel.samples[1].timestamp);
    CHECK_EQUAL(10000, wheel.samples[2].timestamp);
    CHECK_EQUAL(42, wheel.samples[0].value);
    CHECK_EQUAL(42, wheel.samples[1].value);
    CHECK_EQUAL(42, wheel.samples[2].value);
}

TEST(Wheel, WheelUpdatePostInit)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 142);
    wheel_update(&wheel, sample2);

    CHECK_EQUAL(10000, wheel.samples[0].timestamp);
    CHECK_EQUAL(10000, wheel.samples[1].timestamp);
    CHECK_EQUAL(20000, wheel.samples[2].timestamp);
    CHECK_EQUAL(42, wheel.samples[0].value);
    CHECK_EQUAL(42, wheel.samples[1].value);
    CHECK_EQUAL(142, wheel.samples[2].value);
    CHECK_EQUAL(100, wheel.delta_pos_accumulator);

    odometry_encoder_sample_t sample3;
    encoder_record_sample(&sample3, 30000, 242);
    wheel_update(&wheel, sample3);

    CHECK_EQUAL(10000, wheel.samples[0].timestamp);
    CHECK_EQUAL(20000, wheel.samples[1].timestamp);
    CHECK_EQUAL(30000, wheel.samples[2].timestamp);
    CHECK_EQUAL(42, wheel.samples[0].value);
    CHECK_EQUAL(142, wheel.samples[1].value);
    CHECK_EQUAL(242, wheel.samples[2].value);
    CHECK_EQUAL(200, wheel.delta_pos_accumulator);
}

TEST(Wheel, WheelPredictNothing)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 142);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 30000, 242);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 40000, 342);
    wheel_update(&wheel, sample2);

    wheel_predict(&wheel, 40000);

    CHECK_EQUAL(20000, wheel.samples[0].timestamp);
    CHECK_EQUAL(30000, wheel.samples[1].timestamp);
    CHECK_EQUAL(40000, wheel.samples[2].timestamp);
    CHECK_EQUAL(142, wheel.samples[0].value);
    CHECK_EQUAL(242, wheel.samples[1].value);
    CHECK_EQUAL(342, wheel.samples[2].value);
}

TEST(Wheel, WheelPredictIdlePosition)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 42);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 30000, 42);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 40000, 42);
    wheel_update(&wheel, sample2);

    wheel_predict(&wheel, 40000);

    CHECK_EQUAL(20000, wheel.samples[0].timestamp);
    CHECK_EQUAL(30000, wheel.samples[1].timestamp);
    CHECK_EQUAL(40000, wheel.samples[2].timestamp);
    CHECK_EQUAL(42, wheel.samples[0].value);
    CHECK_EQUAL(42, wheel.samples[1].value);
    CHECK_EQUAL(42, wheel.samples[2].value);
}

TEST(Wheel, WheelPredictPositiveVel)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 142);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 30000, 242);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 40000, 342);
    wheel_update(&wheel, sample2);

    uint16_t prediction = wheel_predict(&wheel, 50000);

    CHECK_EQUAL(442, prediction);
}

TEST(Wheel, WheelPredictNegativeVel)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 342);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 30000, 242);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 40000, 142);
    wheel_update(&wheel, sample2);

    uint16_t prediction = wheel_predict(&wheel, 50000);

    CHECK_EQUAL(42, prediction);
}

TEST(Wheel, WheelPredictPositiveAcc)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 142);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 30000, 242);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 40000, 442);
    wheel_update(&wheel, sample2);

    uint16_t prediction = wheel_predict(&wheel, 50000);

    CHECK_EQUAL(742, prediction);
}

TEST(Wheel, WheelPredictNegativeAcc)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 142);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 30000, 442);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 40000, 642);
    wheel_update(&wheel, sample2);

    uint16_t prediction = wheel_predict(&wheel, 50000);

    CHECK_EQUAL(742, prediction);
}

TEST(Wheel, WheelGetDeltaTicksZero)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 42);
    wheel_update(&wheel, sample2);

    int16_t delta = wheel_get_delta_tick(&wheel, 20000);

    CHECK_EQUAL(0, delta);
}

TEST(Wheel, WheelGetDeltaTicksPositive)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 142);
    wheel_update(&wheel, sample2);

    int16_t delta = wheel_get_delta_tick(&wheel, 20000);

    CHECK_EQUAL(100, delta);
}

TEST(Wheel, WheelGetDeltaTicksNegative)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 0);
    wheel_update(&wheel, sample2);

    int16_t delta = wheel_get_delta_tick(&wheel, 20000);

    CHECK_EQUAL(-42, delta);
}

TEST(Wheel, WheelGetDeltaTicksPredictPositive)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 142);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 30000, 242);
    wheel_update(&wheel, sample2);

    int16_t delta = wheel_get_delta_tick(&wheel, 40000);

    CHECK_EQUAL(300, delta);
}

TEST(Wheel, WheelGetDeltaTicksNegativePositive)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 22);
    wheel_update(&wheel, sample2);
    encoder_record_sample(&sample2, 30000, 2);
    wheel_update(&wheel, sample2);

    int16_t delta = wheel_get_delta_tick(&wheel, 40000);

    CHECK_EQUAL(-60, delta);
}

TEST(Wheel, WheelGetDeltaMetersZero)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 42);
    wheel_update(&wheel, sample2);

    float delta = wheel_get_delta_meter(&wheel, 20000);

    DOUBLES_EQUAL(0.0f, delta, 1e-7);
}

TEST(Wheel, WheelGetDeltaMetersPositive)
{
    odometry_encoder_sample_t sample2;
    encoder_record_sample(&sample2, 20000, 142);
    wheel_update(&wheel, sample2);

    float delta = wheel_get_delta_meter(&wheel, 20000);

    DOUBLES_EQUAL(4.79369e-3, delta, 1e-9);
}


TEST_GROUP(Base)
{
    odometry_differential_base_t robot;

    void setup(void)
    {
        robot_base_pose_2d_s init_pose;
        init_pose.x = 0.0f;
        init_pose.y = 0.0f;
        init_pose.theta = 0.0f;

        base_init(&robot, init_pose, 0.5f, 0.5f, 1.0f, 0);
    }
};

TEST(Base, BaseInit)
{
    DOUBLES_EQUAL(0.0f, robot.pose.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.theta, 1e-7);

    DOUBLES_EQUAL(0.0f, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.omega, 1e-7);

    DOUBLES_EQUAL(1.0f, robot.wheelbase, 1e-7);
    CHECK_EQUAL(0, robot.time_last_estim);
}

TEST(Base, BaseUpdateIdle)
{
    odometry_encoder_sample_t sample0;
    encoder_record_sample(&sample0, 10000, 42);
    base_update(&robot, sample0, sample0);

    DOUBLES_EQUAL(0.0f, robot.pose.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.theta, 1e-7);

    DOUBLES_EQUAL(0.0f, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.omega, 1e-7);
}

TEST(Base, BaseUpdateCstPositiveFwdSpeed)
{
    odometry_encoder_sample_t sample0;
    encoder_record_sample(&sample0, 10000, 42);
    base_update(&robot, sample0, sample0);
    encoder_record_sample(&sample0, 20000, 142);
    base_update(&robot, sample0, sample0);
    encoder_record_sample(&sample0, 30000, 242);
    base_update(&robot, sample0, sample0);

    DOUBLES_EQUAL(9.587379924e-3, robot.pose.x, 1e-9);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.theta, 1e-7);

    DOUBLES_EQUAL(0.479368996, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.omega, 1e-7);
}

TEST(Base, BaseUpdateCstNegativeFwdSpeed)
{
    odometry_encoder_sample_t sample0;
    encoder_record_sample(&sample0, 10000, 242);
    base_update(&robot, sample0, sample0);
    encoder_record_sample(&sample0, 20000, 142);
    base_update(&robot, sample0, sample0);
    encoder_record_sample(&sample0, 30000, 42);
    base_update(&robot, sample0, sample0);

    DOUBLES_EQUAL(-9.587379924e-3, robot.pose.x, 1e-9);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.theta, 1e-7);

    DOUBLES_EQUAL(-0.479368996, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.omega, 1e-7);
}

TEST(Base, BaseUpdateCstPositiveRotSpeed)
{
    odometry_encoder_sample_t sample_r;
    odometry_encoder_sample_t sample_l;
    encoder_record_sample(&sample_r, 10000, 242);
    encoder_record_sample(&sample_l, 10000, 242);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 20000, 342);
    encoder_record_sample(&sample_l, 20000, 142);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 30000, 442);
    encoder_record_sample(&sample_l, 30000, 42);
    base_update(&robot, sample_r, sample_l);

    DOUBLES_EQUAL(0.0f, robot.pose.x, 1e-9);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(1.917475984e-2, robot.pose.theta, 1e-8);

    DOUBLES_EQUAL(0.0f, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.958737992, robot.velocity.omega, 1e-7);
}

TEST(Base, BaseUpdateCstNegativeRotSpeed)
{
    odometry_encoder_sample_t sample_r;
    odometry_encoder_sample_t sample_l;
    encoder_record_sample(&sample_r, 10000, 242);
    encoder_record_sample(&sample_l, 10000, 242);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 20000, 142);
    encoder_record_sample(&sample_l, 20000, 342);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 30000, 42);
    encoder_record_sample(&sample_l, 30000, 442);
    base_update(&robot, sample_r, sample_l);

    DOUBLES_EQUAL(0.0f, robot.pose.x, 1e-9);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(-1.917475984e-2, robot.pose.theta, 1e-8);

    DOUBLES_EQUAL(0.0f, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(-0.958737992, robot.velocity.omega, 1e-7);
}

TEST(Base, BaseUpdateCstPositiveFwdAcc)
{
    odometry_encoder_sample_t sample_r;
    odometry_encoder_sample_t sample_l;
    encoder_record_sample(&sample_r, 10000, 242);
    encoder_record_sample(&sample_l, 10000, 242);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 20000, 342);
    encoder_record_sample(&sample_l, 20000, 342);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 30000, 542);
    encoder_record_sample(&sample_l, 30000, 542);
    base_update(&robot, sample_r, sample_l);

    DOUBLES_EQUAL(1.4381069e-2, robot.pose.x, 1e-9);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.theta, 1e-8);

    DOUBLES_EQUAL(0.958737992, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.omega, 1e-7);
}

TEST(Base, BaseUpdateCstNegativeFwdAcc)
{
    odometry_encoder_sample_t sample_r;
    odometry_encoder_sample_t sample_l;
    encoder_record_sample(&sample_r, 10000, 242);
    encoder_record_sample(&sample_l, 10000, 242);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 20000, 442);
    encoder_record_sample(&sample_l, 20000, 442);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 30000, 542);
    encoder_record_sample(&sample_l, 30000, 542);
    base_update(&robot, sample_r, sample_l);

    DOUBLES_EQUAL(1.4381069e-2, robot.pose.x, 1e-9);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.theta, 1e-8);

    DOUBLES_EQUAL(0.479368996, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.omega, 1e-7);
}

TEST(Base, BaseUpdateCstNegativeFwdAccAsyncWheels)
{
    odometry_encoder_sample_t sample_r;
    odometry_encoder_sample_t sample_l;
    encoder_record_sample(&sample_r, 10000, 242);
    encoder_record_sample(&sample_l, 10000, 242);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 20000, 442);
    encoder_record_sample(&sample_l, 20000, 442);
    base_update(&robot, sample_r, sample_l);
    encoder_record_sample(&sample_r, 30000, 542);
    encoder_record_sample(&sample_l, 26000, 514);
    base_update(&robot, sample_r, sample_l);

    DOUBLES_EQUAL(1.4381069e-2, robot.pose.x, 1e-9);
    DOUBLES_EQUAL(0.0f, robot.pose.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.pose.theta, 1e-8);

    DOUBLES_EQUAL(0.479368996, robot.velocity.x, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.y, 1e-7);
    DOUBLES_EQUAL(0.0f, robot.velocity.omega, 1e-7);
}
