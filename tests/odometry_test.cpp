#include "CppUTest/TestHarness.h"

extern "C" {
#include "../src/odometry.h"
}

TEST_GROUP(Wheel)
{
	encoder_sample_t init_state;
	wheel_odom_t wheel;

	void setup(void)
	{
		init_state.timestamp = 20;
		init_state.value = 42;
		wheel_init(&wheel, init_state, 100, 1.0f, 0.5f);
	}
};

TEST(Wheel, WheelInit)
{
	wheel_odom_t wheelp;
	wheel_init(&wheelp, init_state, 100, 1.0f, 0.5f);

	CHECK_EQUAL(init_state.timestamp, wheelp.samples[0].timestamp);
	CHECK_EQUAL(init_state.timestamp, wheelp.samples[1].timestamp);
	CHECK_EQUAL(init_state.timestamp, wheelp.samples[2].timestamp);
	CHECK_EQUAL(init_state.value, wheelp.samples[0].value);
	CHECK_EQUAL(init_state.value, wheelp.samples[1].value);
	CHECK_EQUAL(init_state.value, wheelp.samples[2].value);
	CHECK_EQUAL(100, wheelp.encoder_max);
	DOUBLES_EQUAL(0.5f, wheelp.radius, 1e-7);
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

TEST(Wheel, WheelGetAccZero)
{
	wheel_update(&wheel, 25, 10);
	wheel_update(&wheel, 30, 10);
	wheel_update(&wheel, 35, 10);
	float acc = wheel_get_acc(&wheel);

	DOUBLES_EQUAL(0.0f, acc, 1e-5);
}

TEST(Wheel, WheelGetAccPositive)
{
	wheel_update(&wheel, 25, 10);
	wheel_update(&wheel, 30, 20);
	wheel_update(&wheel, 35, 40);
	float acc = wheel_get_acc(&wheel);

	DOUBLES_EQUAL(0.4f, acc, 1e-5);
}

TEST(Wheel, WheelGetAccNegative)
{
	wheel_update(&wheel, 25, 10);
	wheel_update(&wheel, 30, 30);
	wheel_update(&wheel, 35, 40);
	float acc = wheel_get_acc(&wheel);

	DOUBLES_EQUAL(-0.4f, acc, 1e-5);
}

TEST(Wheel, GetDerivativeZero)
{
	float df = get_time_derivative(0, 10, 1, 10);

	DOUBLES_EQUAL(0.0f, df, 1e-5);
}

TEST(Wheel, GetDerivativePositive)
{
	float df = get_time_derivative(0, 10, 1, 11);

	DOUBLES_EQUAL(1.0f, df, 1e-5);
}

TEST(Wheel, GetDerivativeNegative)
{
	float df = get_time_derivative(0, 10, 1, 9);

	DOUBLES_EQUAL(-1.0f, df, 1e-5);
}

// TEST(Wheel, GetDeltaPositive)
// {
// 	uint32_t delta = get_positive_delta(10, 20, 30);

// 	CHECK_EQUAL(10, delta);
// }

// TEST(Wheel, GetDeltaOverflow)
// {
// 	uint32_t delta = get_positive_delta(10, 5, 30);

// 	CHECK_EQUAL(25, delta);
// }

// TEST(Wheel, GetDeltaOutOfRange)
// {
// 	uint32_t delta = get_positive_delta(10, 35, 30);

// 	CHECK_EQUAL(25, delta);
// }

TEST(Wheel, EncoderRecord)
{
	encoder_sample_t sample;
	encoder_record_sample(&sample, 20, 42);

	CHECK_EQUAL(20, sample.timestamp);
	CHECK_EQUAL(42, sample.value);
}
