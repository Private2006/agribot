#!/usr/bin/env python3
import math

import numpy as np

import rclpy
from rclpy.constants import S_TO_NS
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


class FourWheelDifferentialController(Node):

    def __init__(self):
        super().__init__("four_wheel_diff_controller")
        
        # Parameters for a differential drive model, now applied to a 4-wheeled base
        self.declare_parameter("wheel_radius", 0.15001) # Whhen I doubled the size of the robot I forgot to change here
        self.declare_parameter("wheel_separation", 0.47146) #But the robot still moves fine

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius {self.wheel_radius_:.3f}")
        self.get_logger().info(f"Using track width {self.wheel_separation_:.3f}")
        
        # to calculate change in wheel position
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        # to calculate change in time
        self.prev_time_ = self.get_clock().now()
        
        # Placeholders for position and orientation of the robot
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0


        # Publisher for the 4 wheel velocity commands
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        
        # Subscriber to the robot's desired linear (x) and angular (z) velocity
        self.vel_sub_ = self.create_subscription(TwistStamped, "agribot_diff_controller/cmd_vel", self.velCallback, 10)
        
        # This acts like the simulated verison of a wheel encoder. I gives the positions of the wheel
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)    
        
        self.odom_pub_ = self.create_publisher(Odometry, "agribot_controller/odom", 10)
  
        
        self.forward_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                             [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        
        self.get_logger().info(f"The Forward Kinematics matrix is:\n{self.forward_conversion_}")
        
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0
        
        self.tf_broadcaster_ = TransformBroadcaster(self)



    def velCallback(self, msg):
        """
        Calculates the required angular wheel speeds from the robot's desired linear (x) and angular (z) velocity.
        """
        # Desired robot linear and angular speed
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        # Calculate the required wheel speeds (Left and Right)
        # Using the inverse of the Forward Kinematics matrix: M_IK = inv(M_FK)
        # Note: The wheel speeds calculated here are angular speeds (rad/s).
        wheel_speed_ang = np.matmul(np.linalg.inv(self.forward_conversion_), robot_speed) 
              
        right_speed_lin = wheel_speed_ang[1, 0]
        left_speed_lin = wheel_speed_ang[0, 0] # I swapped the left and roght here, let's see what happens. It worked. The AI misinterpreted the matrix

        # ---------------------------------------------------------------
        # 4-Wheel specific part: Map the left/right speeds to four wheels
        # Assuming the controller expects the order: [Front Right, Rear Right, Front Left, Rear Left]
        # All wheels on the left side move at the same speed.
        # All wheels on the right side move at the same speed.
        
        wheel_speed_msg = Float64MultiArray()

        wheel_speed_msg.data = [
            right_speed_lin,   # Front right 
            right_speed_lin,   # Rear right
            left_speed_lin,    # Front left
            left_speed_lin     # Rear left
        ]
        # ---------------------------------------------------------------
        
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
        # self.get_logger().info(f"Published speeds: {wheel_speed_msg.data}")
        
    def jointCallback(self, msg):
        dp_left = msg.position[2] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_
        
        # Calculate velocities
        dt_sec = dt.nanoseconds / S_TO_NS
        
        # Avoid division by zero
        if dt_sec < 1e-6:
            return 
        
        fi_left = dp_left / dt_sec
        fi_right = dp_right / dt_sec
        
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        
        # ✅ CRITICAL: Update previous values for next callback
        self.left_wheel_prev_pos_ = msg.position[2]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)
        
        # Calculate odometry increments using position deltas (not velocities!)
        d_s = (self.wheel_radius_ / 2) * (dp_right + dp_left)
        d_theta = (self.wheel_radius_ / self.wheel_separation_) * (dp_right - dp_left)
        
        # Update pose
        self.theta_ += d_theta
        
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)
        
        # Create quaternion
        q = quaternion_from_euler(0, 0, self.theta_)
        
        # Populate odometry message
        current_time = self.get_clock().now().to_msg()
        self.odom_msg_.header.stamp = current_time
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        
        self.odom_pub_.publish(self.odom_msg_)
        
        # ← NEW: Broadcast TF transform
        self.broadcast_tf(current_time, q)
        
        # self.get_logger().info(f"Linear: {linear:.3f}, Angular: {angular:.3f}")
        # self.get_logger().info(f"x: {self.x_:.3f}, y: {self.y_:.3f}, theta: {self.theta_:.3f}")

    def broadcast_tf(self, timestamp, quaternion):
        """Broadcast the transform from odom to base_footprint"""
        transform = TransformStamped()
        
        # Header
        transform.header.stamp = timestamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        
        # Translation
        transform.transform.translation.x = self.x_
        transform.transform.translation.y = self.y_
        transform.transform.translation.z = 0.0
        
        # Rotation
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        # Broadcast the transform
        self.tf_broadcaster_.sendTransform(transform)

        
        

def main(args=None):
    rclpy.init(args=args)

    four_wheel_diff_controller = FourWheelDifferentialController()
    rclpy.spin(four_wheel_diff_controller)
    
    four_wheel_diff_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()