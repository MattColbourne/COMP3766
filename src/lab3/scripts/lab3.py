#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

def compute_ik(position, orientation):
    """
    Function that calulates the PUMA analytical IK function.
    Should take a 3x1 position vector and a 3x3 rotation matrix,
    and return a list of joint positions.
    """
    
    print("\nReceived Position:")
    print(position)

    print("\nReceived Orientation (3x3 Rotation Matrix):")
    print(orientation)

    # PUMA Robot Parameters (meters)
    d1, a2, a3 = 0.150, 0.432, 0.432  # Given parameters
    
    # Replace with the actual analytical IK computation
    # Insert you code here
    
    # Comment this line when you have your solution
    theta1, theta2, theta3, theta4, theta5, theta6 = [0.0] * 6

    joint_positions = np.array([theta1, theta2, theta3, theta4, theta5, theta6])

    return joint_positions

def pose_callback(msg):
    """
    Callback function to handle incoming end-effector pose messages.
    You probably do not have to change this
    """
    # Extract position (3x1)
    position = np.array([msg.position.x, msg.position.y, msg.position.z])

    # Extract orientation (3x3 rotation matrix from quaternion)
    q = msg.orientation
    orientation = np.array([
        [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
        [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
        [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]
    ])

    # Compute inverse kinematics
    joint_positions = compute_ik(position, orientation)

    # Publish joint states
    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
    joint_msg.position = joint_positions
    joint_pub.publish(joint_msg)

if __name__ == "__main__":
    rospy.init_node("ik_solver_node", anonymous=True)

    # Publisher: sends joint positions
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    
    print("\nWaiting for /goal_pose")
    
    # Subscriber: listens to end-effector pose
    rospy.Subscriber("/goal_pose", Pose, pose_callback)

    rospy.spin()
