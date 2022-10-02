#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

/**
 * The space frame is defined at the base of joint_1, where its orientation is
 * such that the x-axis points to the front, y-axis to the left, and z-axis to
 * the top of the robot. The SPACE_FRAME_RPY macro is the roll, pitch, yaw of
 * this defined space frame to your space frame by applying the respective
 * rotations in radians. As a recap, RPY corresponds to the rotation about XYZ
 * in that order. If your space frame is defined in the same way, the array
 * should be all zero. The relative position of the space frames can be ignored.
 */
#define SPACE_FRAME_RPY     { 0, 0, 0 }

/**
 * If you are using different units to metres for pose position, then adjust
 * this macro to scale from metres to your position units. E.g., for millimeters
 * change macro to `1000`. Leave at 1 if metres is used.
 */
#define METRE_TO_POS_UNITS  1

/**
 * Topic subscribed by inverse kinematics.
 * Must be of type `geometry_msgs/Pose`.
 */
#define DESIRED_POSE_TOPIC  "/desired_pose"

/**
 * Topic published by dynamixel interface.
 * Must be of type `sensor_msgs/JointState`.
 */
#define JOINT_STATES_TOPIC  "/joint_states"

/**
 * For debugging purposes.
 * Set this to 1 if you want to run the program standalone.
 */
#define DRY_RUN             1

#endif//__CONFIG_HPP__