#ifndef __MILESTONE_HPP__
#define __MILESTONE_HPP__

/////////////////////////////////
// Uncomment to enter debug mode.
// #define DEBUG
/////////////////////////////////

#include "metr4202_milestone/config.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include "Eigen/Dense"

#define N_JOINTS        4

#define MOVE_THRESHOLD  0.02

#define EXIT_ROUTINE    2
#define EXIT_SHUTDOWN   3
#define EXIT_BAD_TUTOR  4

#define HINT(x)         ROS_INFO("[ HINT ]: %s", x);
#define PASS(x)         ROS_INFO("[\033[1;32m PASS \033[0m]: %s", x)
#define FAIL(x)         ROS_INFO("[\033[1;31m FAIL \033[0m]: %s", x)

using namespace geometry_msgs;
using namespace sensor_msgs;

struct Milestone
{
    void joint_states_callback(const JointState::ConstPtr &msg);
    bool is_set_to(const std::vector<double> &pos,
                   const std::array<double, N_JOINTS> &set);
    bool is_moving(const std::vector<double> &pos);
    std::vector<double> prev_pos;
    std::vector<double> ref_pos;
    std::size_t counter;

    static void set_task1a_pose(Pose &pose);
    static void set_task1b_pose(Pose &pose);
    static void set_task1c_pose(Pose &pose);
    static void set_task2a_pose(Pose &pose);
    static void set_task2b_pose(Pose &pose);
    static void set_task2c_pose(Pose &pose);
    static void set_task3a_pose(Pose &pose);
    static void set_task3b_pose(Pose &pose);
    static void set_task3c_pose(Pose &pose);
};

enum class Task
{
    // zero configuration
    homing, 
    // workspace test
    task1a,
    task1b,
    task1c,
    // inv. kin. test
    task2a,
    task2b,
    task2c,
    // collision test
    task3a,
    task3b,
    task3c,
    // complete test
    complete
};

extern Task TASK;

extern bool SYNC;

extern bool COMPLETE;

#endif//__MILESTONE_HPP__