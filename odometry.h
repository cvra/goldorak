#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "timestamp/timestamp.h"
#include "robot_base.h"


/** Encoder sample */
typedef struct {
	timestamp_t timestamp;
	uint16_t value;
} odometry_encoder_sample_t;

/** Encoder wheel
 *
 * @note Contains wheel info needed for odometry, ie. previous
 * encoder samples and the wheel radius.
 */
typedef struct {
	odometry_encoder_sample_t samples[3];
	float delta_pos_accumulator;
	float tick_to_meter;
} odometry_wheel_t;

/** Differential drive base
 *
 * @note Has two encoder wheels associated & geometrical parameters
 * @note Encoder max is always 2^16
 */
typedef struct {
	struct robot_base_pose_2d_s *pose;
	struct robot_base_vel_2d_s *velocity;
	odometry_wheel_t *right_wheel;
	odometry_wheel_t *left_wheel;
	float wheelbase;
	timestamp_t time_last_estim;
} odometry_differential_base_t;


void base_init(
		odometry_differential_base_t *robot,
		const struct robot_base_pose_2d_s init_pose,
		const float right_wheel_radius,
		const float left_wheel_radius,
		const float wheelbase,
		const timestamp_t time_now);

void base_update(
		odometry_differential_base_t *robot,
		const odometry_encoder_sample_t right_wheel_sample,
		const odometry_encoder_sample_t left_wheel_sample);

void wheel_init(
		odometry_wheel_t *wheel,
		const float wheel_radius);

void wheel_update(
		odometry_wheel_t *wheel,
		const odometry_encoder_sample_t new_sample);

int16_t wheel_predict(
		odometry_wheel_t *wheel,
		const timestamp_t time_now);

float encoder_time_derivative(
		odometry_encoder_sample_t sample1,
		odometry_encoder_sample_t sample2);

void encoder_record_sample(
		odometry_encoder_sample_t *sample,
		const timestamp_t timestamp,
		const uint32_t value);

#endif
