#include "metr4202_milestone/milestone.hpp"

void Milestone::joint_states_callback(const JointState::ConstPtr &msg)
{
    Task prev_task = TASK;
    bool moving = is_moving(msg->position);

    switch (TASK) {
        // zero configuration
        case Task::homing:
            if (!is_set_to(msg->position, { 0, 0, 0, 0 })) {
                return;
            }
            TASK = Task::task1a;
            break;

        // workspace test
        case Task::task1a:
            if (counter > 5) {
                FAIL("Task timed out after 5s");
                TASK = Task::task1b;
            } else
            if (counter == 0) {
                if (!moving) {
                    FAIL("Did not move to valid pose");
                    TASK = Task::task1b;
                }
            }
            else {
                if (!moving) {
                    PASS("Moved for valid pose");
                    TASK = Task::task1b;
                }
            }
            counter++;
            break;

        case Task::task1b:
            if (moving) {
                FAIL("Moved for invalid pose");
                TASK = Task::task1c;
            } else
            if (counter > 3) {
                PASS("Did not move for invalid pose");
                TASK = Task::task1c;
            }
            ref_pos = msg->position;
            counter++;
            break;

        case Task::task1c:
            if (counter > 3) {
                if (moving) {
                    if ((msg->position[1] > 0 && ref_pos[1] < 0) ||
                        (msg->position[1] < 0 && ref_pos[1] > 0)) {
                        PASS("Handled by fancy manoeuvre :)");
                        TASK = Task::task2a;
                    }
                    else {
                        FAIL("Did not handle motor range of (-150,150) deg");
                        TASK = Task::task2a;
                    }
                }
                else {
                    PASS("Ignored pose entirely");
                    TASK = Task::task2a;
                }
            }
            counter++;
            break;

        // inv. kin. test
        case Task::task2a:
            if (!moving) {
                PASS("Continue with trajectory");
                TASK = Task::task2b;
                COMPLETE = true;
            }
            ref_pos = msg->position;
            break;

        case Task::task2b:
            if (!moving) {
                std::array<double, N_JOINTS> target;
                std::copy(
                    ref_pos.cbegin(),
                    ref_pos.cend(),
                    target.begin()
                );
                target[1] *= -1;
                if (is_set_to(msg->position, target)) {
                    PASS("Continue with trajectory");
                }
                else {
                    FAIL("End-effector should move in a straight line");
                }
                TASK = Task::task2c;
            }
            break;

        case Task::task2c:
            if (!moving) {
                PASS("Did you enjoy the line?");
                TASK = Task::task3a;
            }
            break;

        // collision test
        case Task::task3a:
            if (moving) {
                FAIL("Imminent impact!");
                TASK = Task::task3b;
            } else
            if (counter > 3) {
                PASS("Did not smack itself on the ground");
                TASK = Task::task3b;
            }
            counter++;
            break;

        case Task::task3b:
            if (moving) {
                FAIL("Imminent impact!");
                TASK = Task::task3c;
            } else
            if (counter > 3) {
                PASS("Did not smack itself on the wooden wall");
                TASK = Task::task3c;
            }
            counter++;
            break;

        case Task::task3c:
            if (moving) {
                FAIL("Imminent impact!");
                TASK = Task::task1c;
            } else
            if (counter > 3) {
                PASS("Did not smack itself");
                TASK = Task::task1c;
            }
            counter++;
            break;

        // complete test
        case Task::complete:
            ROS_ERROR("Something unexpected has happened...");
            ROS_ERROR("Please report this to the tutors!");
            break;
    }
    
    COMPLETE = (prev_task != TASK);
    prev_pos = std::move(msg->position);
}

bool Milestone::is_set_to(const std::vector<double> &pos,
                          const std::array<double, N_JOINTS> &set)
{
    for (std::size_t i = 0; i < N_JOINTS; i++) {

        if (std::abs(pos[i] - set[i]) > MOVE_THRESHOLD) {
            return false;
        }
    }
    return true;
}

bool Milestone::is_moving(const std::vector<double> &pos)
{

    for (std::size_t i = 0; i < N_JOINTS; i++) {
        
        if (std::abs(pos[i] - prev_pos[i]) > MOVE_THRESHOLD) {
            return true;
        }
    }
    return false;
}

void Milestone::set_task1a_pose(Pose &pose)
{
    pose.position.x =  0.2;
    pose.position.y =  0.0;
    pose.position.z =  0.2;
}

void Milestone::set_task1b_pose(Pose &pose)
{
    pose.position.x =  0.0;
    pose.position.y =  0.0;
    pose.position.z =  1.0;
}

void Milestone::set_task1c_pose(Pose &pose)
{
    pose.position.x = -0.2;
    pose.position.y =  0.0;
    pose.position.z =  0.2;
}

void Milestone::set_task2a_pose(Pose &pose)
{
    pose.position.x =  0.2;
    pose.position.y =  0.2;
    pose.position.z =  0.2;
}

void Milestone::set_task2b_pose(Pose &pose)
{
    pose.position.x =  0.2;
    pose.position.y = -0.2;
    pose.position.z =  0.2;
}

void Milestone::set_task2c_pose(Pose &pose)
{
    pose.position.x =  0.2;
    pose.position.y =  0.0;
    pose.position.z =  0.2;
}

void Milestone::set_task3a_pose(Pose &pose)
{
    pose.position.x =  0.0;
    pose.position.y =  0.2;
    pose.position.z = -0.2;
}

void Milestone::set_task3b_pose(Pose &pose)
{
    pose.position.x =  0.1;
    pose.position.y =  0.1;
    pose.position.z =  0.0;
}

void Milestone::set_task3c_pose(Pose &pose)
{
    pose.position.x =  0.0;
    pose.position.y =  0.0;
    pose.position.z =  0.2;
}