#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "odometry.h"

#define MINIMUM_DELTA_T 1e-7 // To avoid dividing by zero in derivative
#define TICKS_PER_TURN  16384


static void wheel_init(
        odometry_wheel_t *wheel,
        const float wheel_radius);

static void wheel_update(
        odometry_wheel_t *wheel,
        const odometry_encoder_sample_t new_sample);

static uint16_t wheel_predict(
        odometry_wheel_t *wheel,
        const uint32_t time_now);

static int16_t wheel_get_delta_tick(
        odometry_wheel_t *wheel,
        const uint32_t time_now);

static float wheel_get_delta_meter(
        odometry_wheel_t *wheel,
        const uint32_t time_now);

static float wheel_get_radius(
        odometry_wheel_t *wheel);

static void wheel_set_radius(
        odometry_wheel_t *wheel,
        const float new_radius);

static float timestamp_duration_s(
        uint32_t t1,
        uint32_t t2);


void odometry_base_init(
        odometry_differential_base_t *robot,
        const struct robot_base_pose_2d_s init_pose,
        const float right_wheel_radius,
        const float left_wheel_radius,
        const int right_wheel_direction,
        const int left_wheel_direction,
        const float wheelbase,
        const uint32_t time_now)
{
    robot->pose.x = init_pose.x;
    robot->pose.y = init_pose.y;
    robot->pose.theta = init_pose.theta;

    robot->velocity.x = 0.0f;
    robot->velocity.y = 0.0f;
    robot->velocity.omega = 0.0f;

    int right_wheel_dir, left_wheel_dir;

    if(right_wheel_direction >= 0.0f) {
        right_wheel_dir = 1.0f;
    } else {
        right_wheel_dir = -1.0f;
    }
    if(left_wheel_direction >= 0.0f) {
        left_wheel_dir = 1.0f;
    } else {
        left_wheel_dir = -1.0f;
    }

    wheel_init(&robot->right_wheel, right_wheel_radius * right_wheel_dir);
    wheel_init(&robot->left_wheel, left_wheel_radius * left_wheel_dir);

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
    uint32_t time_now;
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

void odometry_state_override(
        odometry_differential_base_t *robot,
        const struct robot_base_pose_2d_s new_state,
        const uint32_t time_now)
{
    robot->pose.x = new_state.x;
    robot->pose.y = new_state.y;
    robot->pose.theta = new_state.theta;

    robot->velocity.x = 0.0f;
    robot->velocity.y = 0.0f;
    robot->velocity.omega = 0.0f;

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

void odometry_base_get_parameters(
        odometry_differential_base_t *robot,
        float *wheelbase,
        float *right_wheel_radius,
        float *left_wheel_radius)
{
    *wheelbase = robot->wheelbase;
    *right_wheel_radius = wheel_get_radius(&robot->right_wheel);
    *left_wheel_radius = wheel_get_radius(&robot->left_wheel);
}

void odometry_base_set_parameters(
        odometry_differential_base_t *robot,
        const float wheelbase,
        const float right_wheel_radius,
        const float left_wheel_radius)
{
    robot->wheelbase = wheelbase;
    wheel_set_radius(&robot->right_wheel, right_wheel_radius);
    wheel_set_radius(&robot->left_wheel, left_wheel_radius);
}

static void wheel_init(
        odometry_wheel_t *wheel,
        const float wheel_radius)
{
    wheel->radius = wheel_radius;
    wheel->tick_to_meter = (2 * M_PI * wheel_radius / TICKS_PER_TURN);
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
        const uint32_t time_now)
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
        const uint32_t time_now)
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
        const uint32_t time_now)
{
    int16_t delta;
    delta = wheel_get_delta_tick(wheel, time_now);
    return delta * wheel->tick_to_meter;
}

static float wheel_get_radius(
        odometry_wheel_t *wheel)
{
    return wheel->radius;
}

static void wheel_set_radius(
        odometry_wheel_t *wheel,
        const float new_radius)
{
    wheel->tick_to_meter *= (new_radius / wheel->radius);
    wheel->radius = new_radius;
}

void odometry_encoder_record_sample(
        odometry_encoder_sample_t *sample,
        const uint32_t timestamp,
        const uint32_t value)
{
    sample->timestamp = timestamp;
    sample->value = value;
}

static float timestamp_duration_s(uint32_t t1, uint32_t t2)
{
    int32_t delta_us = t2 - t1;
    return (float)(delta_us / 1000000.f);
}
