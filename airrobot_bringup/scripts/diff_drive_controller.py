#!/usr/bin/env python3

from math import sin, cos, pi
import rclpy
from rclpy.node import Node

# Import Messages
from geometry_msgs.msg import Twist
from airrobot_msgs.msg import WheelVel



class Controller(Node):
  def __init__(self):
    #initialized odometry node name
    super().__init__('diff_drive_controller')
    self.nodename = "diff_drive_controller"
    self.get_logger().info(f"-I- {self.nodename} started")

    # subscriptions
    self.create_subscription(Twist, "cmd_vel", self.controller_callback, 10)
    self.vel_setpoint_pub = self.create_publisher(WheelVel, "vel_setpoint", 10)
    
    
    # get robot hardware specific parameter
    self.L = self.declare_parameter("robot_wheel_separation_distance", 0.187).value # in meter 
    self.R = self.declare_parameter("robot_wheel_radius", 0.0425).value # in meter
    self.rate = self.declare_parameter("rate", 50).value # raw velocity pub rate
    self.timeout_idle = self.declare_parameter('timeout_idle', 2)

    #set up variable initial value
    self.target_v = 0.0
    self.target_w = 0.0
    self.time_prev_update = self.get_clock().now()

    self.get_logger().info(f"-I- {self.time_prev_update } init current time")


  def controller_callback(self,msg):
    # read incoming msg
    self.target_v = msg.linear.x
    self.target_w = msg.angular.z
    
    # Suppose we have a target velocity v and angular velocity w
    # Suppose we have a robot with wheel radius R and distance between wheels L
    # Let vr and vl be angular wheel velocity for right and left wheels, respectively
    # Relate 2v = R (vr +vl) because the forward speed is the sum of the combined wheel velocities
    # Relate Lw = R (vr - vl) because rotation is a function of counter-clockwise wheel speeds
    # Compute vr = (2v + wL) / 2R
    # Compute vl = (2v - wL) / 2R
    vr = (2*self.target_v + self.target_w*self.L) / (2)
    vl = (2*self.target_v - self.target_w*self.L) / (2)

    # next, we'll publish the controller velocity to motor driver
    vel_setpoint = WheelVel()
    vel_setpoint.left_wheel = vl
    vel_setpoint.right_wheel = vr
    self.vel_setpoint_pub.publish(vel_setpoint)

def main(args=None):
  rclpy.init(args=args)
  try:
      node = Controller()
      rclpy.spin(node)
  except rclpy.exceptions.ROSInterruptException:
      pass

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main(); 


