#include <unistd.h>
#include <iostream>
#include <cstdio>
#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#define ADC_RESOLUTION  4096
#define ADC_VOLTAGE     1.8f

#define LASER1D_ADC_PIN 1
#define LASER1D_RATE    100

void adc_init()
{
    FILE *cape_slots_file;
    cape_slots_file = fopen("/sys/devices/bone_capemgr.*/slots", "w");
    fprintf(cape_slots_file, "BB-ADC");
}

float adc_read(int pin)
{
    float value;
    FILE *adc_file;
    char adc_path[48];
    sprintf(adc_path, "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw", pin);

    adc_file = fopen(adc_path, "r");

    fscanf(adc_file, "%f", &value);
    value /= (ADC_VOLTAGE * ADC_RESOLUTION);

    fclose(adc_file);

    return value;
}

int main(int argc, char **argv)
{
    adc_init();
    ros::init(argc, argv, "laser1d_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::Range>("laser1d/range", 100);
    ros::Rate loop_rate(LASER1D_RATE);

    sensor_msgs::Range msg;

    while (ros::ok()) {
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "laser";

        msg.range = adc_read(LASER1D_ADC_PIN);
        msg.min_range = 0.0;
        msg.max_range = 4.0;
        msg.field_of_view = 0.01;

        pub.publish(msg);
        ROS_DEBUG("Laser 1D range measured: %f", msg.range);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
