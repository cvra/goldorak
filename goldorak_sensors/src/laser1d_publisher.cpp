#include <unistd.h>
#include <iostream>
#include <signal.h>

#include "mraa.hpp"

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#define LASER1D_ADC_PIN 1
#define LASER1D_RATE    100


int main(int argc, char **argv)
{
    mraa::Aio laser1d_adc(LASER1D_ADC_PIN);

    ros::init(argc, argv, "laser1d_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::Range>("laser1d/range", 100);
    ros::Rate loop_rate(LASER1D_RATE);

    sensor_msgs::Range msg;

    while (ros::ok()) {
        msg.header.stamp = ros::Time::now();
        msg.range = laser1d_adc.readFloat();

        pub.publish(msg);
        ROS_DEBUG("Laser 1D range measured: %f", msg.range);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
