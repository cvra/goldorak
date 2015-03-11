#ifndef ROBOT_BASE_H_
#define ROBOT_BASE_H_

/** Pose of a robot in a 2D environment */
struct robot_base_pose_2d_s {
	float x;
	float y;
	float theta;
};

/** Velocity of a robot in a 2D environment */
struct robot_base_vel_2d_s {
	float x;
	float y;
	float omega;
};

#endif
