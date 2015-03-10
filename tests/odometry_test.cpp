#include "CppUTest/TestHarness.h"

extern "C" {
#include "../src/odometry.h"
}

TEST_GROUP(Encoder)
{

};

TEST(Encoder, EncoderRecord)
{
	encoder_sample_t sample;
	encoder_record_sample(&sample, 20, 42);

	CHECK_EQUAL(20, sample.timestamp);
	CHECK_EQUAL(42, sample.value);
}


TEST_GROUP(Derivative)
{

};

TEST(Derivative, GetDerivativeZero)
{
	float df = get_time_derivative(0, 10, 1000, 10);

	DOUBLES_EQUAL(0.0f, df, 1e-5);
}

TEST(Derivative, GetDerivativePositive)
{
	float df = get_time_derivative(0, 10, 1000, 11);

	DOUBLES_EQUAL(1e3, df, 1e-4);
}

TEST(Derivative, GetDerivativeNegative)
{
	float df = get_time_derivative(0, 10, 1000, 9);

	DOUBLES_EQUAL(-1e3, df, 1e-4);
}


TEST_GROUP(Wheel)
{
	encoder_sample_t init_state;
	wheel_odom_t wheel;

	void setup(void)
	{
		init_state.timestamp = 20;
		init_state.value = 42;
		wheel_init(&wheel, init_state, 100, 0.5f);
	}
};

TEST(Wheel, WheelInit)
{
	CHECK_EQUAL(init_state.timestamp, wheel.samples[0].timestamp);
	CHECK_EQUAL(init_state.timestamp, wheel.samples[1].timestamp);
	CHECK_EQUAL(init_state.timestamp, wheel.samples[2].timestamp);
	CHECK_EQUAL(init_state.value, wheel.samples[0].value);
	CHECK_EQUAL(init_state.value, wheel.samples[1].value);
	CHECK_EQUAL(init_state.value, wheel.samples[2].value);
	CHECK_EQUAL(100, wheel.encoder_max);
	DOUBLES_EQUAL(0.5f, wheel.radius, 1e-7);
	DOUBLES_EQUAL(0.03141592f, wheel.tick_to_meter, 1e-7);
}

TEST(Wheel, WheelUpdate)
{
	wheel_update(&wheel, 25, 10);
	wheel_update(&wheel, 30, 20);
	wheel_update(&wheel, 35, 40);

	CHECK_EQUAL(25, wheel.samples[0].timestamp);
	CHECK_EQUAL(30, wheel.samples[1].timestamp);
	CHECK_EQUAL(35, wheel.samples[2].timestamp);
	CHECK_EQUAL(10, wheel.samples[0].value);
	CHECK_EQUAL(20, wheel.samples[1].value);
	CHECK_EQUAL(40, wheel.samples[2].value);
}

TEST(Wheel, WheelGetSample)
{
	encoder_sample_t sample;
	wheel_get_sample(&sample, &wheel, 0);

	CHECK_EQUAL(sample.timestamp, wheel.samples[0].timestamp);
	CHECK_EQUAL(sample.value, wheel.samples[0].value);
}

TEST(Wheel, WheelGetVelZero)
{
	wheel_update(&wheel, 25000, 10);
	wheel_update(&wheel, 30000, 10);
	wheel_update(&wheel, 35000, 10);
	float velp = wheel_get_vel(&wheel, 35);
	float velt = wheel_get_vel(&wheel, 40);

	DOUBLES_EQUAL(0.0f, velp, 1e-5);
	DOUBLES_EQUAL(0.0f, velt, 1e-5);
}

TEST(Wheel, WheelGetVelPositive)
{
	wheel_update(&wheel, 25000, 11);
	wheel_update(&wheel, 30000, 11);
	wheel_update(&wheel, 35000, 12);
	float velp = wheel_get_vel(&wheel, 35000);
	float velt = wheel_get_vel(&wheel, 40000);

	DOUBLES_EQUAL(6.283184f, velp, 1e-5);
	DOUBLES_EQUAL(12.566368f, velt, 1e-5);
}

TEST(Wheel, WheelGetVelNegative)
{
	wheel_update(&wheel, 25000, 11);
	wheel_update(&wheel, 30000, 11);
	wheel_update(&wheel, 35000, 10);
	float velp = wheel_get_vel(&wheel, 35000);
	float velt = wheel_get_vel(&wheel, 40000);

	DOUBLES_EQUAL(-6.283184f, velp, 1e-5);
	DOUBLES_EQUAL(-12.566368f, velt, 1e-5);
}

TEST(Wheel, WheelGetAccZero)
{
	wheel_update(&wheel, 25000, 10);
	wheel_update(&wheel, 30000, 10);
	wheel_update(&wheel, 35000, 10);
	float acc = wheel_get_acc(&wheel);

	DOUBLES_EQUAL(0.0f, acc, 1e-5);
}

TEST(Wheel, WheelGetAccPositive)
{
	wheel_update(&wheel, 25000, 10);
	wheel_update(&wheel, 30000, 20);
	wheel_update(&wheel, 35000, 40);
	float acc = wheel_get_acc(&wheel);

	DOUBLES_EQUAL(12566.37f, acc, 1e-2);
}

TEST(Wheel, WheelGetAccNegative)
{
	wheel_update(&wheel, 25000, 10);
	wheel_update(&wheel, 30000, 30);
	wheel_update(&wheel, 35000, 40);
	float acc = wheel_get_acc(&wheel);

	DOUBLES_EQUAL(-12566.37f, acc, 1e-2);
}


TEST_GROUP(Base)
{
	base_odom_t robot;
	wheel_odom_t r_wheel;
	wheel_odom_t l_wheel;
	encoder_sample_t init_state;
	float x0[3];
	timestamp_t t0;

	void setup(void)
	{
		x0[0] = 0.0f;
		x0[1] = 0.0f;
		x0[2] = 0.0f;
		t0 = 1000;
		init_state.timestamp = 20;
		init_state.value = 42;

		base_init(&robot, 1.0f, 1.0f, 1.0f, x0, t0);
		wheel_init(&(robot.right_wheel), init_state, 100, 0.5f);
		wheel_init(&(robot.left_wheel), init_state, 100, 0.5f);
	}
};

TEST(Base, BaseInit)
{
	DOUBLES_EQUAL(1.0f, robot.wheelbase, 1e-7);
	DOUBLES_EQUAL(1.0f, robot.vel_max, 1e-7);
	DOUBLES_EQUAL(1.0f, robot.acc_max, 1e-7);
	DOUBLES_EQUAL(x0[0], robot.pose[0], 1e-7);
	DOUBLES_EQUAL(x0[1], robot.pose[1], 1e-7);
	DOUBLES_EQUAL(x0[2], robot.pose[2], 1e-7);
	DOUBLES_EQUAL(0.0f, robot.vel[0], 1e-7);
	DOUBLES_EQUAL(0.0f, robot.vel[1], 1e-7);
	DOUBLES_EQUAL(0.0f, robot.acc[0], 1e-7);
	DOUBLES_EQUAL(0.0f, robot.acc[1], 1e-7);
	CHECK_EQUAL(t0, robot.time_last_estim);
}

TEST(Base, BaseEstimateVelZero)
{
	wheel_update(&(robot.left_wheel), 25000, 10);
	wheel_update(&(robot.left_wheel), 30000, 10);
	wheel_update(&(robot.left_wheel), 35000, 10);

	wheel_update(&(robot.right_wheel), 25000, 10);
	wheel_update(&(robot.right_wheel), 30000, 10);
	wheel_update(&(robot.right_wheel), 35000, 10);

	base_estim_vel(&robot, 35000);

	DOUBLES_EQUAL(0.0f, robot.vel[0], 1e-7);
	DOUBLES_EQUAL(0.0f, robot.vel[1], 1e-7);
}
