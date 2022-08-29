#include "metr4202_milestone/milestone.hpp"

JointState joint;
Pose pose;

void kill(int help);
void ritual(void);
void parse(void);
void callback(const JointState::ConstPtr msg);

#define MAIN(x) ROS_ERROR("[main]: %s", x)
int main(int argc, char **argv)
{
    ros::init(argc, argv, "milestone");
    ros::NodeHandle node;

    ritual();

    ros::Subscriber sub = node.subscribe(DESIRED_JOINT_STATES, 10, callback);
    ros::Publisher pub = node.advertise<Pose>(DESIRED_POSE, 10);
    ros::Rate rate(FREQ);

    ROS_INFO("Milestone ready");

    while (node.ok()) {

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void kill(int help)
{
    if (help) {
        ROS_FATAL("There's been a problem... Please ask a tutor for help!");
    }
    exit(EXIT_FAILURE);
}

#define RITUAL(x) ROS_ERROR("[ritual]: %s", x)
void ritual(void)
{
    ros::master::V_TopicInfo topics;

    if (!ros::master::getTopics(topics)) {
        RITUAL("Unexpected error: Could not query topics from master");
        kill(1);
    }
    if (topics.size() == 0) {
        RITUAL("Topic error: Please make sure to run your robot stack first");
        kill(0);
    }

    ros::master::TopicInfo joint, pose;

    for (auto &t : topics) {
        if (t.name == DESIRED_JOINT_STATES) {
            joint = t;
        } else if (t.name == DESIRED_POSE) {
            pose = t;
        }
    }

    std::string exTopic = "('" + std::string(DESIRED_JOINT_STATES) + \
                            "', 'sensor_msgs/JointState'), " + \
                          "('" + std::string(DESIRED_POSE) + \
                            "', 'geometry_msgs/Pose')";

    if (joint.name.empty()) {
        RITUAL("Topic error: Please make sure to run your dynamixel interface first");
        RITUAL("Topic error: Expecting " + exTopic);
        kill(0);
    }
    if (joint.datatype != "sensor_msgs/JointState") {
        RITUAL("Topic error: I know you changed the message type... Change it back please");
        RITUAL("Topic error: Expecting " + exTopic);
        kill(0);
    }
    if (pose.name.empty()) {
        RITUAL("Topic error: Expecting " + exTopic);
        kill(1);
    }
    if (strcmp(pose.datatype.c_str(), "geometry_msgs/Pose") > 0) {
        RITUAL("Topic error: Detected type variant of 'goemetry_msgs/Pose'");
        RITUAL("Topic error: Expecting " + exTopic);
        kill(1);
    }
    if (pose.datatype != "geometry_msgs/Pose") {
        RITUAL("Topic error: Expecting " + exTopic);
        kill(1);
    }
}

#define PARSE(x) ROS_ERROR("[parse]: %s", x)
void parse(void)
{
    XmlRpc::XmlRpcValue ports, servos;

    try {
        ros::param::get("~ports", ports);

        if (ports.size() != 1) {
            PARSE("Port error: Interesting... Do you need more than one port?");
            PARSE("Port error: Check your controller_config.yaml file");
            kill(0);
        }
    } catch (...) {
        PARSE("Port error: Caught exception when parsing ports");
        PARSE("Port error: Check the dynamixel_interface for errors");
        kill(1);
    }

    try {
        servos = ports[0]["servos"];

        if (servos.size() != 4) {
            PARSE("Servo error: I belive we gave you a 4-DOF robot... Might want to check that");
            PARSE("Servo error: Check your controller_config.yaml file");
            kill(0);
        }

        for (std::size_t i = 0; i < 4; i++) {


        }
    } catch (...) {

    }
}

void callback(const JointState::ConstPtr msg)
{
    joint = *msg;
}