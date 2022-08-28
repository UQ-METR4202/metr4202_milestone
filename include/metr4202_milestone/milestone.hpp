#ifndef MILESTONE_HPP
#define MILESTONE_HPP

#include "metr4202_milestone/config.hpp"

#include "ros/ros.h"
#include "ros/master.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"

using namespace sensor_msgs;
using namespace geometry_msgs;

extern JointState joint;
extern Pose pose;

/**
 * Publishes desired end-effector pose outside of the workspace.
 */
void check_workspace();

/**
 * Publishes desired end-effector pose in a circular trajectory.
 */
void check_kinematic();

/**
 * Publishes desired end-effector pose in invalid taskspace.
 */
void check_taskspace();

/**
 * Calculates the forward kinematics of the robot and returns the end-effector pose.
 */
Pose forward_kinematics();

#endif//MILESTONE_HPP