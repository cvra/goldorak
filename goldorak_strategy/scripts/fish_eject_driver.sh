#!/bin/bash

PWM_DIR=/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip*/

config-pin 8.46 pwm

echo 1 > $PWM_DIR/unexport
echo 1 > $PWM_DIR/export
echo 1 > $PWM_DIR/pwm1/enable
echo 20000000 > $PWM_DIR/pwm1/period
echo $1 > $PWM_DIR/pwm1/duty_cycle
