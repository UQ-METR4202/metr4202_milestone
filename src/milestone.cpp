#include "metr4202_milestone/milestone.hpp"

void Milestone::joint_states_callback(const JointState::ConstPtr &msg)
{
    if (!SYNC) return;

    Task prev_task = TASK;
    bool moving = is_moving(msg->position);

#ifdef DEBUG
    std::cout << "position=" << msg->position[0] << ','
                             << msg->position[1] << ','
                             << msg->position[2] << ','
                             << msg->position[3] << '\n';
    std::cout << "is_moving=" << moving << '\n';
#endif

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
                TASK = Task::complete;
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
            if (!moving) {
                if (counter == 0) {
                    PASS("Ignored pose entirely");
                    TASK = Task::task2a;
                }
                else {
                    if ((msg->position[1] > 0 && ref_pos[1] < 0) ||
                        (msg->position[1] < 0 && ref_pos[1] > 0)) {
                        PASS("Here's an Easter egg -> ()");
                        TASK = Task::task2a;
                    }
                    else {
                        FAIL("Did not handle motor range of (-150,150) deg");
                        TASK = Task::task2a;
                    }
                }
            }
            counter++;
            break;

        // inv. kin. test
        case Task::task2a:
            PASS("Continuing");
            TASK = Task::task2b;
            break;

        case Task::task2b:
            PASS("Continuing");
            TASK = Task::task2c;
            break;

        case Task::task2c:
            PASS("Did it move in a straight line?");
            TASK = Task::task3a;
            break;

        // collision test
        case Task::task3a:
            if (moving) {
                FAIL("Imminent impact! Stopping test early...");
                TASK = Task::complete;
            } else
            if (counter > 3) {
                PASS("Did not smack itself on the ground");
                TASK = Task::task3b;
            }
            counter++;
            break;

        case Task::task3b:
            if (moving) {
                FAIL("Imminent impact! Stopping test early...");
                TASK = Task::complete;
            } else
            if (counter > 3) {
                PASS("Did not smack itself on the conveyor wall");
                TASK = Task::task3c;
            }
            counter++;
            break;

        case Task::task3c:
            if (moving) {
                FAIL("Imminent impact!");
                TASK = Task::complete;
            } else
            if (counter > 3) {
                PASS("Did not smack itself");
                TASK = Task::complete;
            }
            counter++;
            break;

        // complete test
        case Task::complete:
            ROS_ERROR("Something unexpected has happened...");
            ROS_ERROR("Please report this to the tutors!");
            break;
    }
    
    SYNC = false;
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
    pose.position.x =  0.1;
    pose.position.y =  0.0;
    pose.position.z =  0.1;
}

void Milestone::set_task1b_pose(Pose &pose)
{
    pose.position.x =  0.1;
    pose.position.y =  0.5;
    pose.position.z =  0.1;
}

void Milestone::set_task1c_pose(Pose &pose)
{
    pose.position.x = -0.1;
    pose.position.y =  0.0;
    pose.position.z =  0.1;
}

void Milestone::set_task2a_pose(Pose &pose)
{
    pose.position.x =  0.07;
    pose.position.y =  0.07;
    pose.position.z =  0.1;
}

void Milestone::set_task2b_pose(Pose &pose)
{
    pose.position.x =  0.07;
    pose.position.y =  0.0;
    pose.position.z =  0.1;
}

void Milestone::set_task2c_pose(Pose &pose)
{
    pose.position.x =  0.07;
    pose.position.y = -0.07;
    pose.position.z =  0.1;
}

void Milestone::set_task3a_pose(Pose &pose)
{
    pose.position.x =  0.0;
    pose.position.y =  0.1;
    pose.position.z = -0.1;
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
    pose.position.z =  0.1;
}