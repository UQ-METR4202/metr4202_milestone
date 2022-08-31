#ifndef MILESTONE_HPP
#define MILESTONE_HPP

#include "metr4202_milestone/config.hpp"

#include <ros/ros.h>
#include <ros/master.h
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace sensor_msgs;
using namespace geometry_msgs;

struct Servo
{
    int min;
    int max;
    int zero;
};

extern JointState g_joint;
extern Servo g_servos[4];

/**
 * Publishes desired end-effector pose outside of the workspace.
 */
bool check_workspace(ros::Publisher &pub);

/**
 * Publishes desired end-effector pose in colliding objects.
 */
bool check_collision(ros::Publisher &pub);


/**
 * Publishes desired end-effector pose in a circular trajectory.
 */
bool check_kinematic(ros::Publisher &pub);

/**
 * Calculates the forward kinematics of the robot and returns the end-effector pose.
 */
Pose forward_kinematics();

#endif//MILESTONE_HPP