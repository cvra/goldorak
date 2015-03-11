#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "timestamp/timestamp.h"
#include "odometry.h"

#define MINIMUM_DELTA_T 1e-7 // To avoid dividing by zero in derivative


void base_init(
		odometry_differential_base_t *robot,
		const float wheelbase,
		const struct robot_base_pose_2d_s init_pose,
		const timestamp_t time_now)
{

}

void base_update(
		odometry_differential_base_t *robot,
		const odometry_encoder_sample_t right_wheel_sample,
		const odometry_encoder_sample_t left_wheel_sample,
		const timestamp_t time_now)
{

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
		const timestamp_t new_timestamp,
		const uint32_t new_value)
{
	wheel->samples[0].timestamp = wheel->samples[1].timestamp;
	wheel->samples[0].value = wheel->samples[1].value;

	wheel->samples[1].timestamp = wheel->samples[2].timestamp;
	wheel->samples[1].value = wheel->samples[2].value;

	wheel->samples[2].timestamp = new_timestamp;
	wheel->samples[2].value = new_value;
}

void wheel_predict(
		odometry_wheel_t *wheel,
		const timestamp_t time_now)
{

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
