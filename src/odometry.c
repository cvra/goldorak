#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "timestamp/timestamp.h"
#include "odometry.h"

void base_init(
		base_odom_t *robot,
		const float wheelbase,
		const float vel_max,
		const float acc_max,
		const float init_pose[3],
		const timestamp_t time_now)
{
	robot->wheelbase = wheelbase;
	robot->vel_max = vel_max;
	robot->acc_max = acc_max;

	robot->pose[0] = init_pose[0];
	robot->pose[1] = init_pose[1];
	robot->pose[2] = init_pose[2];

	robot->vel[0] = 0.0f;
	robot->vel[1] = 0.0f;

	robot->acc[0] = 0.0f;
	robot->acc[1] = 0.0f;

	robot->time_last_estim = time_now;
}

void base_estim_pose(
		base_odom_t *robot,
		timestamp_t time_now)
{
	base_estim_vel(robot, time_now);

	robot->pose[2] += robot->vel[1] \
					  * timestamp_duration_s(robot->time_last_estim, time_now);
	robot->pose[0] += robot->vel[0] * cos(robot->pose[2]) \
					  * timestamp_duration_s(robot->time_last_estim, time_now);
	robot->pose[1] += robot->vel[0] * sin(robot->pose[2]) \
					  * timestamp_duration_s(robot->time_last_estim, time_now);

	robot->time_last_estim = time_now;
}

void base_estim_vel(
		base_odom_t *robot,
		timestamp_t time_now)
{
	float right_wheel_vel = wheel_get_vel(&(robot->right_wheel), time_now);
	float left_wheel_vel = wheel_get_vel(&(robot->left_wheel), time_now);

	// Linear interpolation: Constant acceleration
	base_estim_acc(robot);
	float lin_vel = (right_wheel_vel + left_wheel_vel) / 2.0f;
	float ang_vel = 0.5f * (right_wheel_vel - left_wheel_vel) / robot->wheelbase;

	lin_vel += robot->acc[0] * timestamp_duration_s(robot->time_last_estim, \
													time_now);
	ang_vel += robot->acc[1] * timestamp_duration_s(robot->time_last_estim, \
													time_now);

	// Check velocity is within boundaries
	if(lin_vel < robot->vel_max) {
		robot->vel[0] = lin_vel;
	} else {
		robot->vel[0] = robot->vel_max;
	}

	if(ang_vel < (robot->vel_max / robot->wheelbase)) {
		robot->vel[1] = ang_vel;
	} else {
		robot->vel[1] = robot->vel_max / robot->wheelbase;
	}
}

void base_estim_acc(
		base_odom_t *robot)
{
	float right_wheel_acc = wheel_get_acc(&(robot->right_wheel));
	float left_wheel_acc = wheel_get_acc(&(robot->left_wheel));

	float lin_acc = (right_wheel_acc + left_wheel_acc) / 2.0f;
	float ang_acc = (right_wheel_acc - left_wheel_acc) / robot->wheelbase;

	// Check acceleration is within boundaries
	if(lin_acc < robot->acc_max) {
		robot->acc[0] = lin_acc;
	} else {
		robot->acc[0] = robot->acc_max;
	}

	if(ang_acc < (2.0f * robot->acc_max / robot->wheelbase)) {
		robot->acc[1] = ang_acc;
	} else {
		robot->acc[1] = 2.0f * robot->acc_max / robot->wheelbase;
	}
}

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
