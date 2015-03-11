#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "timestamp/timestamp.h"
#include "odometry.h"

#define MINIMUM_DELTA_T 1e-7 // To avoid dividing by zero in derivative


void base_init(
		odometry_differential_base_t *robot,
		const struct robot_base_pose_2d_s init_pose,
		const float right_wheel_radius,
		const float left_wheel_radius,
		const float wheelbase,
		const timestamp_t time_now)
{
	robot->pose->x = init_pose.x;
	robot->pose->y = init_pose.y;
	robot->pose->theta = init_pose.theta;

	robot->velocity->x = 0.0f;
	robot->velocity->y = 0.0f;
	robot->velocity->omega = 0.0f;

	wheel_init(robot->right_wheel, right_wheel_radius);
	wheel_init(robot->left_wheel, left_wheel_radius);

	robot->wheelbase = wheelbase;
	robot->time_last_estim = time_now;
}

void base_update(
		odometry_differential_base_t *robot,
		const odometry_encoder_sample_t right_wheel_sample,
		const odometry_encoder_sample_t left_wheel_sample)
{
	wheel_update(robot->right_wheel, right_wheel_sample);
	wheel_update(robot->left_wheel, left_wheel_sample);

	// Choose wheel with most recent timestamp as reference
	timestamp_t time_now;
	if (0.0f <= timestamp_duration_s(right_wheel_sample.timestamp,
									 left_wheel_sample.timestamp)) {
		time_now = right_wheel_sample.timestamp;
	} else {
		time_now = left_wheel_sample.timestamp;
	}

	// Wheel prediction
	int16_t delta_right_wheel = wheel_predict(robot->right_wheel, time_now);
	int16_t delta_left_wheel = wheel_predict(robot->left_wheel, time_now);

	// Base prediction
	float dt = timestamp_duration_s(robot->time_last_estim, time_now);
	float delta_fwd = 0.5f * (delta_right_wheel + delta_left_wheel);
	float delta_rot = (delta_right_wheel - delta_left_wheel) / robot->wheelbase;

	robot->velocity->x = delta_fwd
						 * cos(robot->pose->theta + 0.5f * delta_rot) / dt;
	robot->velocity->y = delta_fwd
						 * sin(robot->pose->theta + 0.5f * delta_rot) / dt;
	robot->velocity->omega = delta_rot / dt;

	robot->pose->x += delta_fwd * cos(robot->pose->theta + 0.5f * delta_rot);
	robot->pose->y += delta_fwd * sin(robot->pose->theta + 0.5f * delta_rot);
	robot->pose->theta += delta_rot;

	robot->time_last_estim = time_now;
}

void wheel_init(
		odometry_wheel_t *wheel,
		const float wheel_radius)
{
	wheel->tick_to_meter = (2 * M_PI * wheel_radius / 65536);

	wheel->delta_pos_accumulator = NAN;
}

void wheel_update(
		odometry_wheel_t *wheel,
		const odometry_encoder_sample_t new_sample)
{
	// If not initialised, Initialise!
	if (wheel->delta_pos_accumulator == NAN) {
		wheel->samples[0].timestamp = new_sample.timestamp;
		wheel->samples[0].value = new_sample.value;
		wheel->samples[1].timestamp = new_sample.timestamp;
		wheel->samples[1].value = new_sample.value;
		wheel->samples[2].timestamp = new_sample.timestamp;
		wheel->samples[2].value = new_sample.value;

		wheel->delta_pos_accumulator = 0.0f;
	} else {
		wheel->samples[0].timestamp = wheel->samples[1].timestamp;
		wheel->samples[0].value = wheel->samples[1].value;
		wheel->samples[1].timestamp = wheel->samples[2].timestamp;
		wheel->samples[1].value = wheel->samples[2].value;
		wheel->samples[2].timestamp = new_sample.timestamp;
		wheel->samples[2].value = new_sample.value;

		int16_t dvalue = (int16_t) (wheel->samples[2].value \
								    - wheel->samples[1].value);
		wheel->delta_pos_accumulator += dvalue * wheel->tick_to_meter;
	}
}

int16_t wheel_predict(
		odometry_wheel_t *wheel,
		const timestamp_t time_now)
{
	int16_t delta_pos = wheel->delta_pos_accumulator;

	if (wheel->samples[2].timestamp == time_now) {
		wheel->delta_pos_accumulator = 0.0f;
	} else {
		float acc, vel, dt;
		int16_t dvalue_predicted;

		// Compute previous acceleration
		acc = encoder_time_derivative(wheel->samples[1], wheel->samples[2])
			  - encoder_time_derivative(wheel->samples[0], wheel->samples[1]);
		acc /= timestamp_duration_s(wheel->samples[1].timestamp,
								    wheel->samples[2].timestamp);

		// Compute previous velocity
		vel = encoder_time_derivative(wheel->samples[1], wheel->samples[2]);

		// Predict current encoder value
		dt = timestamp_duration_s(wheel->samples[2].timestamp, time_now);
		dvalue_predicted = (int16_t) (vel * dt + 0.5f * acc * dt * dt);

		wheel->delta_pos_accumulator = dvalue_predicted;
		delta_pos += dvalue_predicted;
	}

	return delta_pos;
}

float encoder_time_derivative(
		odometry_encoder_sample_t sample1,
		odometry_encoder_sample_t sample2)
{
	float dt = timestamp_duration_s(sample1.timestamp, sample2.timestamp);
	int16_t dvalue = (int16_t) (sample2.value - sample1.value);

	if (dt >= MINIMUM_DELTA_T) {
		return (float) (dvalue / dt);
	} else {
		return 0.0f;
	}
}

void encoder_record_sample(
		odometry_encoder_sample_t *sample,
		const timestamp_t timestamp,
		const uint32_t value)
{
	sample->timestamp = timestamp;
	sample->value = value;
}
