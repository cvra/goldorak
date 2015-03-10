#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "timestamp/timestamp.h"

// #define US_IN_ONE_SECOND 1000000

/** Instance of an encoder sample */
typedef struct{
	timestamp_t timestamp;
	uint32_t value;
} encoder_sample_t;

/** Instance of a wheel
 *
 * @note Contains wheel info needed for odometry, ie. previous
 * encoder samples and the wheel radius.
 */
typedef struct{
	encoder_sample_t samples[3];
	uint32_t encoder_max;
	float transmission_ratio; // ticks_per_rev = encoder_max / transmission_ratio
	float radius;
} wheel_odom_t;


void wheel_init(
		wheel_odom_t *wheel,
		const encoder_sample_t encoder_init_state,
		const uint32_t encoder_max,
		const float transmission_ratio,
		const float wheel_radius);

void wheel_get_sample(
		encoder_sample_t *sample,
		wheel_odom_t *wheel,
		const uint8_t sample_nb);

void wheel_update(
		wheel_odom_t *wheel,
		const timestamp_t new_timestamp,
		const uint32_t new_value);

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
