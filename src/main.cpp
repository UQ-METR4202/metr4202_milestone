#include "metr4202_milestone/milestone.hpp"

Task TASK;
bool COMPLETE;

int routine(void)
{
    ros::master::V_TopicInfo topics{};

    if (!ros::master::getTopics(topics)) {
        ROS_ERROR("Failed to query topics from master");
        return EXIT_FAILURE;
    }
    if (topics.size() == 0) {
        ROS_ERROR("Run your robot stack first!");
        return EXIT_FAILURE;
    }

    ros::master::TopicInfo pose{}, joint{};

    for (auto &t : topics) {

        if (t.name == JOINT_STATES_TOPIC) {
            joint = t;
        } else
        if (t.name == DESIRED_POSE_TOPIC) {
            pose = t;
        }
    }

#if(DRY_RUN == 1)
    if (pose.name.empty()) {
        ROS_ERROR("Topic name `%s` does not exist!", DESIRED_POSE_TOPIC);
        return EXIT_FAILURE;
    }
#endif
    if (joint.name.empty()) {
        ROS_ERROR("Topic name `%s` does not exist!", JOINT_STATES_TOPIC);
        return EXIT_FAILURE;
    }
#if(DRY_RUN == 1)
    if (pose.datatype != "geometry_msgs/Pose") {
        ROS_ERROR(
            "Expected message type `geometry_msgs/Pose` for topic `%s`",
            JOINT_STATES_TOPIC
        );
        return EXIT_FAILURE;
    }
#endif
    if (joint.datatype != "sensor_msgs/JointState") {
        ROS_ERROR(
            "Expected message type `sensor_msgs/JointState` for topic `%s`",
            JOINT_STATES_TOPIC
        );
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

const std::string task_to_string(void)
{
    switch (TASK) {
        // zero configuration
        case Task::homing:
            ROS_WARN("Undefined homing string!");
            return "";

        // workspace test
        case Task::task1a:
            return " Task 1a: Valid pose in workspace";
        case Task::task1b:
            return " Task 1b: Invalid pose outside of workspace";
        case Task::task1c:
            return " Task 1c: Invalid pose due to motor limitation";

        // inv. kin. test
        case Task::task2a:
            return " Task 2a: End-effector trajectory point 1";
        case Task::task2b:
            return " Task 2b: End-effector trajectory point 2";
        case Task::task2c:
            return " Task 2c: End-effector trajectory point 3";

        // collision test
        case Task::task3a:
            return " Task 3a: Collision with ground";
        case Task::task3b:
            return " Task 3b: Collision with environment";
        case Task::task3c:
            return " Task 3c: Collision with robot links";

        // complete test
        case Task::complete:
            return "Test complete!";
    }
    ROS_WARN("Undefined control path!");
    return "";
}

void transform_pose(Pose &pose)
{
    constexpr double rpy[] = SPACE_FRAME_RPY;

    const Eigen::Matrix3d Rx {
        { 1,                0,                 0 },
        { 0, std::cos(rpy[0]), -std::sin(rpy[0]) },
        { 0, std::sin(rpy[0]),  std::cos(rpy[0]) }
    };
    const Eigen::Matrix3d Ry {
        {  std::cos(rpy[1]), 0, std::sin(rpy[1]) },
        {                 0, 1,                0 },
        { -std::sin(rpy[1]), 0, std::cos(rpy[1]) }
    };
    const Eigen::Matrix3d Rz {
        { std::cos(rpy[2]), -std::sin(rpy[2]), 0 },
        { std::sin(rpy[2]),  std::cos(rpy[2]), 0 },
        {                0,                 0, 1 }
    };

    const Eigen::Matrix3d R = Rx * Ry * Rz;

    Eigen::Vector3d vec { 
        pose.position.x,
        pose.position.y,
        pose.position.z
    };

    vec = R * vec * METRE_TO_POS_UNITS;

    pose.position.x = vec[0];
    pose.position.y = vec[1];
    pose.position.z = vec[2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "milestone");
    ros::NodeHandle node{};
    
    if (routine()) {
        return EXIT_ROUTINE;
    }

    Milestone milestone{};
    milestone.prev_pos = { 0, 0, 0, 0 };
    milestone.ref_pos = { 0, 0, 0, 0 };

    ros::Rate rate(1);

    ros::Publisher pub = node.advertise<Pose>(DESIRED_POSE_TOPIC, 10);

    ros::Subscriber sub = node.subscribe(
        JOINT_STATES_TOPIC,
        10,
        &Milestone::joint_states_callback,
        &milestone
    );

    {
        ros::Publisher pubtmp = node.advertise<JointState>(
            "desired_joint_states", 10
        );

        JointState msg{};

        msg.header.stamp = ros::Time::now();
        msg.name = { "joint_1", "joint_2", "joint_3", "joint_4" };
        msg.position = { 0, 0, 0, 0 };

        pubtmp.publish(msg);

        TASK = Task::homing;

        while (TASK == Task::homing) {
            ROS_INFO("asdf");
            if (!ros::ok()) {
                return EXIT_SHUTDOWN;
            }
            pubtmp.publish(msg);
            // rate.sleep();
            ros::Duration(1).sleep();
            ros::spinOnce();
        }
    }

    Pose pose{};
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    COMPLETE = true;

    std::string tsk_msg;

    ROS_INFO("=======================================================");
    ROS_INFO("             STARTING  METR4202  MILESTONE             ");

    while (ros::ok()) {

        if (COMPLETE) {

            const std::string task_msg = task_to_string();
            ROS_INFO("=======================================================");
            ROS_INFO("%s", task_msg.c_str());
            // ROS_INFO("%s", std::string(task_msg.size(), '-').c_str());
            ROS_INFO("%s", std::string(56, '-').c_str());

            milestone.counter = 0;

            switch (TASK) {
                // zero configuration
                case Task::homing:
                    ROS_ERROR("Something unexpected has happened...");
                    ROS_ERROR("Please report this to the tutors!");
                    return EXIT_BAD_TUTOR;

                // workspace test
                case Task::task1a:
                    tsk_msg = "[ PROG ]: Expecting to move to desired pose...";
                    milestone.set_task1a_pose(pose);
                    break;
                case Task::task1b:
                    tsk_msg =
                        "[ PROG ]: " \
                        "Expecting to handle inverse kinematics error...";
                    milestone.set_task1b_pose(pose);
                    break;
                case Task::task1c:
                    tsk_msg = "[ PROG ]: " \
                        "Expecting to handle motor range limit...";
                    milestone.set_task1c_pose(pose);
                    break;

                // inv. kin. test
                case Task::task2a:
                    tsk_msg = "[ PROG ]: " \
                        "Expecting to move to starting trajectory point...";
                    milestone.set_task2c_pose(pose);
                    break;
                case Task::task2b:
                    tsk_msg = "[ PROG ]: " \
                        "Expecting to move to middle trajectory point...";
                    milestone.set_task2c_pose(pose);
                    break;
                case Task::task2c:
                    tsk_msg = "[ PROG ]: " \
                        "Expecting to move to end trajectory point...";
                    milestone.set_task2c_pose(pose);
                    break;

                // collision test
                case Task::task3a:
                    milestone.set_task3c_pose(pose);
                    break;
                case Task::task3b:
                    milestone.set_task3c_pose(pose);
                    break;
                case Task::task3c:
                    milestone.set_task3c_pose(pose);
                    break;

                // complete test
                case Task::complete:
                    ROS_INFO("Shutting down milestone program...");
                    ros::shutdown();
                    return EXIT_SUCCESS;
            }

            transform_pose(pose);
        }

        ROS_INFO("%s", tsk_msg.c_str());
        pub.publish(pose);
        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_BAD_TUTOR;
}