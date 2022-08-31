#include "metr4202_milestone/milestone.hpp"

JointState g_joint;
Servo g_servos[4];

void kill(int help);
void ritual(void);
void parse(void);
void callback(const JointState::ConstPtr msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "milestone");
    ros::NodeHandle node;
    ROS_INFO("Started node handle...");

    ritual();
    ROS_INFO("Completed ritual...");

    parse();
    ROS_INFO("Parsed controller config...");

    ros::Subscriber sub = node.subscribe(DESIRED_JOINT_STATES, 10, callback);
    ros::Publisher pub = node.advertise<Pose>(DESIRED_POSE, 10);
    ros::Rate rate(FREQ);
    ROS_INFO("Initialised publisher / subscriber...");

    ROS_INFO("=============================================");
    ROS_INFO("               MILESTONE READY               ");
    ROS_INFO("=============================================");

    ROS_INFO("Homing to zero configuration");
    pub.publish(Pose{});

    while ( g_joint.position[0] < -0.1 || g_joint.position[0] > 0.1 ||
            g_joint.position[1] < -0.1 || g_joint.position[1] > 0.1 ||
            g_joint.position[2] < -0.1 || g_joint.position[2] > 0.1 ||
            g_joint.position[3] < -0.1 || g_joint.position[3] > 0.1 ) {
        if (!node.ok()) {
            ROS_WARN("Terminating while attempting to home");
            return 1;
        }
        ROS_INFO("...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Done");

    int task = 0; // 1, 2, 3
    bool check_complete = true;

    while (node.ok()) {
        if (check_complete) {
            check_complete = false;
            switch (task++) {
                case 0:
                    ROS_INFO("=============================================");
                    ROS_INFO("Starting task 1: Checking workspace detection");
                    ROS_INFO("---------------------------------------------");
                    break;

                case 1:
                    ROS_INFO("=============================================");
                    ROS_INFO("Starting task 2: Checking collision detection");
                    ROS_INFO("---------------------------------------------");
                    break;

                case 2:
                    ROS_INFO("=============================================");
                    ROS_INFO("Starting task 3: Checking forward kinematics ");
                    ROS_INFO("---------------------------------------------");
                    break;

                case 3:
                    ROS_INFO("=============================================");
                    ROS_INFO("Milestone complete!");
                    return 0;
            }
        }

        switch (task) {
            case 1:
                check_complete = check_workspace(pub);
                break;

            case 2:
                check_complete = check_collision(pub);
                break;

            case 3:
                check_complete = check_kinematic(pub);
                break;

            default:
                ROS_ERROR("Invalid task number!");
                kill(1);
        }

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

        for (int i = 0; i < 4; i++) {
            auto id = std::to_string(i);

            if (servos[i]["min_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                PARSE("Servo error: Incorrect type min_pos for id " + id);
                kill(0);
            }
            if (servos[i]["max_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                PARSE("Servo error: Incorrect type max_pos for id " + id);
                kill(0);
            }
            if (servos[i]["zero_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                PARSE("Servo error: Incorrect type zero_pos for id " + id);
                kill(0);
            }

            g_servos[i].min = servos[i]["min_pos"];
            g_servos[i].max = servos[i]["max_pos"];
            g_servos[i].zero = servos[i]["zero_pos"];
            
            if (g_servos[i].min > g_zeros[i].min) {
                PARSE("Servo error: min_pos > max_pos for id " + id);
                kill(0);
            }
            if (g_servos[i].zero < g_servos[i].min || g_servos[i].zero > g_servos[i].max) {
                PARSE("Servo error: zero_pos not between min_pos, max_pos for id " + id);
                kill(0);
            }
        }
    } catch (...) {
        PARSE("Servo error: Caught exception when parsing servos");
        PARSE("Servo error: Check the dynamixel_interface for errors");
        kill(1);
    }
}

void callback(const JointState::ConstPtr msg)
{
    g_joint = *msg;
}