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
	wheel_odom_t wheelp;
	wheel_init(&wheelp, init_state, 100, 0.5f);

	CHECK_EQUAL(init_state.timestamp, wheelp.samples[0].timestamp);
	CHECK_EQUAL(init_state.timestamp, wheelp.samples[1].timestamp);
	CHECK_EQUAL(init_state.timestamp, wheelp.samples[2].timestamp);
	CHECK_EQUAL(init_state.value, wheelp.samples[0].value);
	CHECK_EQUAL(init_state.value, wheelp.samples[1].value);
	CHECK_EQUAL(init_state.value, wheelp.samples[2].value);
	CHECK_EQUAL(100, wheelp.encoder_max);
	DOUBLES_EQUAL(0.5f, wheelp.radius, 1e-7);
	DOUBLES_EQUAL(0.03141592f, wheelp.tick_to_meter, 1e-7);
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

	DOUBLES_EQUAL(12566.37f, acc, 1e-1);
}

TEST(Wheel, WheelGetAccNegative)
{
	wheel_update(&wheel, 25000, 10);
	wheel_update(&wheel, 30000, 30);
	wheel_update(&wheel, 35000, 40);
	float acc = wheel_get_acc(&wheel);

	DOUBLES_EQUAL(-12566.37f, acc, 1e-1);
}
