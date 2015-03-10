#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "timestamp/timestamp.h"
#include "odometry.h"

void wheel_init(
		wheel_odom_t *wheel,
		const encoder_sample_t init_state,
		const uint32_t encoder_max,
		const float wheel_radius)
{
	encoder_record_sample(
		&(wheel->samples[0]),
		init_state.timestamp,
		init_state.value);
	encoder_record_sample(
		&(wheel->samples[1]),
		init_state.timestamp,
		init_state.value);
	encoder_record_sample(
		&(wheel->samples[2]),
		init_state.timestamp,
		init_state.value);
	wheel->encoder_max = encoder_max;
	wheel->radius = wheel_radius;
	wheel->tick_to_meter = (2 * M_PI * wheel_radius / encoder_max);
}

void wheel_get_sample(
		encoder_sample_t *sample,
		wheel_odom_t *wheel,
		const uint8_t sample_nb)
{
	timestamp_t timestamp = wheel->samples[sample_nb].timestamp;
	uint32_t value = wheel->samples[sample_nb].value;
	encoder_record_sample(sample, timestamp, value);
}

void wheel_update(
		wheel_odom_t *wheel,
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

float wheel_get_vel(
		wheel_odom_t *wheel,
		const timestamp_t time_now)
{
	float acc = wheel_get_acc(wheel);
	float vel_prev = get_time_derivative(wheel->samples[1].timestamp,
							        	 wheel->samples[1].value,
							  	    	 wheel->samples[2].timestamp,
							  	    	 wheel->samples[2].value);
	float dt = timestamp_duration_s(wheel->samples[2].timestamp, time_now);

	return vel_prev * wheel->tick_to_meter + acc * dt;
}

float wheel_get_acc(
		wheel_odom_t *wheel)
{
	float vel1 = get_time_derivative(wheel->samples[0].timestamp,
						        	 wheel->samples[0].value,
						  	    	 wheel->samples[1].timestamp,
						  	    	 wheel->samples[1].value);
	float vel2 = get_time_derivative(wheel->samples[1].timestamp,
						        	 wheel->samples[1].value,
						  	    	 wheel->samples[2].timestamp,
						  	    	 wheel->samples[2].value);
	float acc = get_time_derivative(wheel->samples[1].timestamp, vel1,
						  	   		wheel->samples[2].timestamp, vel2);
	// Convert acc to meters
	return acc * wheel->tick_to_meter;
}

float get_time_derivative(
		const float t1,
		const float x1,
		const float t2,
		const float x2)
{
	float dt = (float) timestamp_duration_s(t1, t2);

	if(dt >= 1e-7) {
		return (x2 - x1) / dt;
	} else {
		return 0.0f;
	}
}

void encoder_record_sample(
		encoder_sample_t *encoder_sample,
		const timestamp_t timestamp,
		const uint32_t value)
{
	encoder_sample->timestamp = timestamp;
	encoder_sample->value = value;
}
