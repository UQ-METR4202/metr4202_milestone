#include "metr4202_milestone/milestone.hpp"

Pose task_1_pose[3];
Pose task_2_pose[3];
Pose task_3_pose[3];

void milestone_setup(void) {
    Pose pose;
    tf2::Quaternion q;

    // task 1
    q.setRPY(0, 0, 0);
    tf2::convert(q, pose.orientation);
    pose.position.x = L3 + L4;
    pose.position.y = 0;
    pose.position.z = L1 + L2;
    task_1_pose[0] = pose;
    //
    q.setRPY(0, 0, 0);
    tf2::convert(q, pose.orientation);
    pose.position.x = 4 * (L3 + L4);
    pose.position.y = 0;
    pose.position.z = L1 + L2;
    task_1_pose[1] = pose;
    //
    q.setRPY(0, M_PI_2, 0);
    tf2::convert(q, pose.orientation);
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    task_1_pose[2] = pose;

    // task 2

}
    

#define TASK1(x) ROS_WARN("[task-1]: %s", x)
bool check_workspace(ros::Publisher &pub)
{
    static int lvl = 0; // 1, 2, 3
    static bool next = true;

    if (next) {
        next = false;
        switch (lvl++) {
            case 0:


            case 1:

        }
    }


}

#define TASK2(x) ROS_WARN("[task-2]: %s", x)
bool check_collision(ros::Publisher &pub)
{

}

#define TASK3(x) ROS_WARN("[task-3]: %s", x)
bool check_kinematic(ros::Publisher &pub)
{

}

Pose forward_kinematics()
{
}