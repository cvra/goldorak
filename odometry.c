#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "timestamp/timestamp.h"
#include "odometry.h"

#define MINIMUM_DELTA_T 1e-7 // To avoid dividing by zero in derivative


void odometry_base_init(
        odometry_differential_base_t *robot,
        const struct robot_base_pose_2d_s init_pose,
        const float right_wheel_radius,
        const float left_wheel_radius,
        const float wheelbase,
        const timestamp_t time_now)
{
    robot->pose.x = init_pose.x;
    robot->pose.y = init_pose.y;
    robot->pose.theta = init_pose.theta;

    robot->velocity.x = 0.0f;
    robot->velocity.y = 0.0f;
    robot->velocity.omega = 0.0f;

    wheel_init(&robot->right_wheel, right_wheel_radius);
    wheel_init(&robot->left_wheel, left_wheel_radius);

    robot->wheelbase = wheelbase;
    robot->time_last_estim = time_now;
}

void odometry_base_update(
        odometry_differential_base_t *robot,
        const odometry_encoder_sample_t right_wheel_sample,
        const odometry_encoder_sample_t left_wheel_sample)
{
    wheel_update(&robot->right_wheel, right_wheel_sample);
    wheel_update(&robot->left_wheel, left_wheel_sample);

    // Choose wheel with most recent timestamp as reference
    timestamp_t time_now;
    if (0.0f >= timestamp_duration_s(right_wheel_sample.timestamp,
                                     left_wheel_sample.timestamp)) {
        time_now = right_wheel_sample.timestamp;
    } else {
        time_now = left_wheel_sample.timestamp;
    }

    // Wheel prediction
    float delta_right_wheel = wheel_get_delta_meter(&robot->right_wheel, time_now);
    float delta_left_wheel = wheel_get_delta_meter(&robot->left_wheel, time_now);

    // Base prediction
    float dt = timestamp_duration_s(robot->time_last_estim, time_now);
    float delta_fwd = 0.5f * (delta_right_wheel + delta_left_wheel);
    float delta_rot = (delta_right_wheel - delta_left_wheel) / robot->wheelbase;

    float cos_theta = cosf(robot->pose.theta + 0.5f * delta_rot);
    float sin_theta = sinf(robot->pose.theta + 0.5f * delta_rot);

    if (dt >= MINIMUM_DELTA_T) {
        robot->velocity.x = delta_fwd * cos_theta / dt;
        robot->velocity.y = delta_fwd * sin_theta / dt;
        robot->velocity.omega = delta_rot / dt;
    }

    robot->pose.x += delta_fwd * cos_theta;
    robot->pose.y += delta_fwd * sin_theta;
    robot->pose.theta += delta_rot;

    robot->time_last_estim = time_now;
}

void odometry_base_get_pose(
        odometry_differential_base_t *robot,
        struct robot_base_pose_2d_s *pose)
{
    pose->x = robot->pose.x;
    pose->y = robot->pose.y;
    pose->theta = robot->pose.theta;
}

void odometry_base_get_vel(
        odometry_differential_base_t *robot,
        struct robot_base_vel_2d_s *velocity)
{
    velocity->x = robot->velocity.x;
    velocity->y = robot->velocity.y;
    velocity->omega = robot->velocity.omega;
}

static void wheel_init(
        odometry_wheel_t *wheel,
        const float wheel_radius)
{
    wheel->tick_to_meter = (2 * M_PI * wheel_radius / 65536);
    wheel->delta_pos_accumulator = 0;
    wheel->is_initialised = false;
}

static void wheel_update(
        odometry_wheel_t *wheel,
        const odometry_encoder_sample_t new_sample)
{
    // If not initialised, Initialise!
    if (!(wheel->is_initialised)) {
        wheel->samples[0].timestamp = new_sample.timestamp;
        wheel->samples[0].value = new_sample.value;
        wheel->samples[1].timestamp = new_sample.timestamp;
        wheel->samples[1].value = new_sample.value;
        wheel->samples[2].timestamp = new_sample.timestamp;
        wheel->samples[2].value = new_sample.value;

        wheel->delta_pos_accumulator = 0;
        wheel->is_initialised = true;
    } else {
        wheel->samples[0].timestamp = wheel->samples[1].timestamp;
        wheel->samples[0].value = wheel->samples[1].value;
        wheel->samples[1].timestamp = wheel->samples[2].timestamp;
        wheel->samples[1].value = wheel->samples[2].value;
        wheel->samples[2].timestamp = new_sample.timestamp;
        wheel->samples[2].value = new_sample.value;

        wheel->delta_pos_accumulator += (int16_t) (wheel->samples[2].value
                                                  - wheel->samples[1].value);
    }
}

static uint16_t wheel_predict(
        odometry_wheel_t *wheel,
        const timestamp_t time_now)
{
    if (wheel->samples[2].timestamp != time_now) {
        float acc, vel, pos, tau1, tau2, dt1, dt2, dt;
        int16_t dy1, dy2;
        uint16_t prediction;

        // Fit a parabola to the wheel's motion
        dt1 = timestamp_duration_s(wheel->samples[0].timestamp,
                                   wheel->samples[1].timestamp);
        dt2 = timestamp_duration_s(wheel->samples[0].timestamp,
                                   wheel->samples[2].timestamp);
        dy1 = (int16_t) (wheel->samples[1].value - wheel->samples[0].value);
        dy2 = (int16_t) (wheel->samples[2].value - wheel->samples[0].value);

        // Avoid division by zero
        if (dt1 <= MINIMUM_DELTA_T) {
            dt2 = MINIMUM_DELTA_T;
        }
        if (dt2 <= MINIMUM_DELTA_T) {
            dt2 = MINIMUM_DELTA_T;
        }

        tau1 = dy1 / (- dt1 * dt1 + dt1 * dt2);
        tau2 = dy2 / (- dt2 * dt2 + dt1 * dt2);

        acc = - tau1 - tau2;
        vel = tau1 * dt2 + tau2 * dt1;
        pos = wheel->samples[0].value;

        // Predict using fitted parabola
        dt = timestamp_duration_s(wheel->samples[0].timestamp, time_now);
        prediction = pos + (int16_t) (vel * dt + acc * dt * dt);

        return prediction;
    } else {
        return wheel->samples[2].value;
    }
}

static int16_t wheel_get_delta_tick(
        odometry_wheel_t *wheel,
        const timestamp_t time_now)
{
    int16_t delta;
    delta = wheel->delta_pos_accumulator;
    wheel->delta_pos_accumulator = (int16_t) (wheel_predict(wheel, time_now)
                                              - wheel->samples[2].value);
    delta += wheel->delta_pos_accumulator;
    return delta;
}

static float wheel_get_delta_meter(
        odometry_wheel_t *wheel,
        const timestamp_t time_now)
{
    int16_t delta;
    delta = wheel_get_delta_tick(wheel, time_now);
    return delta * wheel->tick_to_meter;
}

void odometry_encoder_record_sample(
        odometry_encoder_sample_t *sample,
        const timestamp_t timestamp,
        const uint32_t value)
{
    sample->timestamp = timestamp;
    sample->value = value;
}
