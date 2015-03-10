#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "timestamp/timestamp.h"

// #define US_IN_ONE_SECOND 1000000

/** Instance of an encoder sample */
typedef struct{
	timestamp_t timestamp;
	uint32_t value;
} encoder_sample_t;

/** Instance of an encoder wheel
 *
 * @note Contains wheel info needed for odometry, ie. previous
 * encoder samples and the wheel radius.
 */
typedef struct{
	encoder_sample_t samples[3];
	uint32_t encoder_max;
	float radius;
	float tick_to_meter;
} wheel_odom_t;

/** Instance of a differential drive base
 *
 * @note Has two encoder wheels associated & geometrical parameters
 */
typedef struct{
	wheel_odom_t right_wheel;
	wheel_odom_t left_wheel;
	float wheelbase;
	float vel_max;
	float acc_max;
	float pose[3]; // x, y, theta
	float vel[2]; // linear_vel, angular_vel
	float acc[2]; // linear_acc, angular_acc
	timestamp_t time_last_estim;
} base_odom_t;


void base_init(
		base_odom_t *robot,
		const float wheelbase,
		const float vel_max,
		const float acc_max,
		const float init_pose[3],
		const timestamp_t time_now);

void base_estim_pose(
		base_odom_t *robot,
		timestamp_t time_now);

void base_estim_vel(
		base_odom_t *robot,
		timestamp_t time_now);

void base_estim_acc(
		base_odom_t *robot);

void wheel_init(
		wheel_odom_t *wheel,
		const encoder_sample_t encoder_init_state,
		const uint32_t encoder_max,
		const float wheel_radius);

void wheel_get_sample(
		encoder_sample_t *sample,
		wheel_odom_t *wheel,
		const uint8_t sample_nb);

void wheel_update(
		wheel_odom_t *wheel,
		const timestamp_t new_timestamp,
		const uint32_t new_value);

float wheel_get_vel(
		wheel_odom_t *wheel,
		const timestamp_t time_now);

float wheel_get_acc(
		wheel_odom_t *wheel);

float get_time_derivative(
		const float t1,
		const float x1,
		const float t2,
		const float x2);

void encoder_record_sample(
		encoder_sample_t *encoder_sample,
		const timestamp_t timestamp,
		const uint32_t value);

#endif
