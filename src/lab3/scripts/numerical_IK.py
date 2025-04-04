#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

def compute_ik(position, orientation, initial_guess, learn_rate=0.05, max_iter=1000, tolerance=1e-6):
    """
    Numerical Inverse Kinematics using Gradient Descent.
    Args:
        position: Target position (3x1 vector).
        orientation: Target orientation (3x3 rotation matrix).
        initial_guess: Initial joint angles (6x1 vector).
        learn_rate: Learning rate for gradient descent.
        max_iter: Maximum number of iterations.
        tolerance: Error tolerance for convergence.

    Returns:
        joint_positions: Optimized joint angles (6x1 vector).
    """
    
    
    def rotation_matrix(axis, angle):
        """
        Compute the rotation matrix for a given axis and angle using Rodrigues' formula.
        Args:
            axis: Axis of rotation (3x1 vector).
            angle: Rotation angle in radians.
        Returns:
            3x3 rotation matrix.
        """
        axis = np.array(axis) / np.linalg.norm(axis)  # Normalize axis
        x, y, z = axis
        c = np.cos(angle)
        s = np.sin(angle)
        t = 1 - c

        return np.array([
            [t*x*x + c, t*x*y - s*z, t*x*z + s*y],
            [t*x*y + s*z, t*y*y + c, t*y*z - s*x],
            [t*x*z - s*y, t*y*z + s*x, t*z*z + c]
        ])

    def forward_kinematics(joint_angles):
        """
        Compute the forward kinematics for a robot given joint angles.
        Args:
            joint_angles: List of joint angles in radians.
        Returns:
            end_effector_position: Position of the end-effector (3x1 vector).
            end_effector_orientation: Orientation of the end-effector (3x3 rotation matrix).
        """
        # Predefined list of joints relative to base frame
        joints = [
            {"position": [0.0, 0.0, 0.67], "axis": [0.0, 0.0, 1.0]},
            {"position": [-0.15, 0.0, 0.0], "axis": [-1.0, 0.0, 0.0]}, 
            {"position": [0.0, 0.432, 0.0], "axis": [-1.0, 0.0, 0.0]}, 
            {"position": [0.0, 0.432, 0.0], "axis": [0.0, 0.0, 1.0]},  
            {"position": [0.0, 0.0, 0.0], "axis": [0.0, 1.0, 0.0]}, 
            {"position": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0]}
        ]

        # Initialize transformation matrix as identity
        T = np.eye(4)

        # Apply joint angles to each joint in order
        for i, joint in enumerate(joints):
            position = np.array(joint["position"])
            axis = np.array(joint["axis"])
            angle = joint_angles[i]  # Use provided joint angle

            # Compute rotation matrix for this joint
            R = rotation_matrix(axis, angle)

            # Create homogeneous transformation matrix for this joint
            T_joint = np.eye(4)
            T_joint[:3, :3] = R  # Rotation part
            T_joint[:3, 3] = position  # Translation part

            # Update total transformation matrix
            T = T @ T_joint

        # Extract end-effector position and orientation from final transformation matrix
        end_effector_position = T[:3, 3]
        end_effector_orientation = T[:3, :3]

        return end_effector_position, end_effector_orientation

    def compute_error(joint_angles):
        # Compute the error between the target and current pose
        current_position, current_orientation = forward_kinematics(joint_angles)
        position_error = np.linalg.norm(position - current_position)
        orientation_error = np.linalg.norm(orientation - current_orientation)
        return position_error + 0.1*orientation_error

    def compute_gradient(joint_angles):
        # Compute numerical gradient of the error with respect to joint angles
        gradient = np.zeros_like(joint_angles)
        delta = 1e-4  # Small perturbation for numerical gradient computation
        for i in range(len(joint_angles)):
            perturbed_angles = joint_angles.copy()
            perturbed_angles[i] += delta
            error_plus = compute_error(perturbed_angles)

            perturbed_angles[i] -= 2 * delta
            error_minus = compute_error(perturbed_angles)

            gradient[i] = (error_plus - error_minus) / (2 * delta)
        return gradient

    # Gradient descent loop
    joint_positions = initial_guess
    for _ in range(max_iter):
        gradient = compute_gradient(joint_positions)
        update = -learn_rate * gradient
        if np.linalg.norm(update) < tolerance:
            break
        joint_positions += update

    return joint_positions

def pose_callback(msg):
    """
    Callback function to handle incoming end-effector pose messages.
    """
    position = np.array([msg.position.x, msg.position.y, msg.position.z])
    q = msg.orientation
    orientation = np.array([
        [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
        [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
        [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]
    ])

    initial_guess = np.zeros(6)  # Initial guess for joint angles
    joint_positions = compute_ik(position, orientation, initial_guess)
    print(joint_positions)

    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
    joint_msg.position = joint_positions
    joint_pub.publish(joint_msg)

if __name__ == "__main__":
    rospy.init_node("ik_solver_node", anonymous=True)
    
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    print("Waiting for goal pose")
    
    rospy.Subscriber("/goal_pose", Pose, pose_callback)

    rospy.spin()
