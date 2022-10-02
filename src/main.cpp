#include "metr4202_milestone/milestone.hpp"

Task TASK;

bool SYNC;

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

    ros::master::TopicInfo joint{};

    for (auto &t : topics) {
        if (t.name == JOINT_STATES_TOPIC) {
            joint = t;
        }
    }

    if (joint.name.empty()) {
        ROS_ERROR("Topic `%s` does not exist!", JOINT_STATES_TOPIC);
        return EXIT_FAILURE;
    }
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
            ROS_ERROR("Undefined homing string!");
            return "";

        // workspace test
        case Task::task1a:
            return " Task 1a: Valid pose in workspace";
        case Task::task1b:
            return " Task 1b: Invalid pose outside of workspace";
        case Task::task1c:
            return " Task 1c: Valid pose outside of motor range";

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
            return " Task 3c: Collision with its own links";

        // complete test
        case Task::complete:
            return " End of test!";
    }
    ROS_ERROR("Undefined control path!");
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

            if (!ros::ok()) {
                return EXIT_SHUTDOWN;
            }
#ifdef DEBUG
            std::cout << "homing...\n";
#endif
            pubtmp.publish(msg);
            rate.sleep();
            SYNC = true;
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

            rate.sleep();

            const std::string task_msg = task_to_string();
            ROS_INFO("=======================================================");
            ROS_INFO("%s", task_msg.c_str());
            ROS_INFO("%s", std::string(55, '-').c_str());

            milestone.counter = 0;

            switch (TASK) {
                // zero configuration
                case Task::homing:
                    ROS_ERROR("Something unexpected has happened...");
                    ROS_ERROR("Please report this to the tutors!");
                    return EXIT_BAD_TUTOR;

                // workspace test
                case Task::task1a:
                    HINT("Ensure your inverse kinematics is fully");
                    HINT("implemented before continuing :)");
                    milestone.set_task1a_pose(pose);
                    break;
                case Task::task1b:
                    HINT("Pose should cause error in inverse kinematics");
                    HINT("soltuion if not handled correctly.");
                    HINT("You should ignore this commanded pose.");
                    milestone.set_task1b_pose(pose);
                    break;
                case Task::task1c:
                    HINT("Dynamixel motors have a range between");
                    HINT("(-150, 150) degrees. You should either ignore");
                    HINT("or handle this case. Remember, there is more");
                    HINT("than one solution.");
                    milestone.set_task1c_pose(pose);
                    break;

                // inv. kin. test
                case Task::task2a:
                    HINT("The inverse kinematics solution itself is not");
                    HINT("tested. You need to verify the solution");
                    HINT("yourself by observing the end-effector is");
                    HINT("drawing a straight line(ish) in Euclidean");
                    HINT("space.");
                    milestone.set_task2a_pose(pose);
                    break;
                case Task::task2b:
                    HINT("The end-effector should move to the middle.");
                    milestone.set_task2b_pose(pose);
                    break;
                case Task::task2c:
                    HINT("The end-effector should move to the end.");
                    milestone.set_task2c_pose(pose);
                    break;

                // collision test
                case Task::task3a:
                    HINT("It is unlikely for your planner node to");
                    HINT("publish a desired pose near the ground,");
                    HINT("but would you want to risk breaking the");
                    HINT("manipulator or losing luggage?");
                    milestone.set_task3a_pose(pose);
                    break;
                case Task::task3b:
                    HINT("Keep in mind the physical design of the rig.");
                    HINT("It may interfere with your trajectory");
                    HINT("planning.");
                    milestone.set_task3b_pose(pose);
                    break;
                case Task::task3c:
                    HINT("An easy way to avoid self-collision is to");
                    HINT("apply soft limits at each joint.");
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

        pub.publish(pose);
        rate.sleep();
        SYNC = true;
        ros::spinOnce();
    }

    return EXIT_SHUTDOWN;
}