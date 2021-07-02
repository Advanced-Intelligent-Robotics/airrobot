#!/usr/bin/env python3

from math import sin, cos, pi
import rclpy
from rclpy.node import Node

# Import Messages
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from airrobot_msgs.msg import WheelVel
from std_msgs.msg import Float64

# Use left and right encoder tick  to compute robot forward kinematics using unicycle model
# Publish the estimated wheel angular velocities, odometry and tf between odom & base_link frame
class Odom(Node):
  def __init__(self):
    #initialized odometry node name
    super().__init__('odometry')
    self.nodename = "odometry"
    self.get_logger().info(f"-I- {self.nodename} started")

    # subscriptions
    self.create_subscription(WheelVel, "wheel_velocity", self.wheel_velocity_callback, 10)
    self.odom_pub = self.create_publisher(Odometry, "odom", 10)
    self.odom_broadcaster = TransformBroadcaster(self)
    
    # get robot hardware specific parameter
    self.L = self.declare_parameter("robot_wheel_separation_distance", 0.187).value # in meter 
    self.R = self.declare_parameter("robot_wheel_radius", 0.0425).value # in meter
    self.dt = self.declare_parameter("dt", 0.02).value # raw velocity pub rate

    # get parent & child frame
    self.frame_id = self.declare_parameter("frame_id","odom").value
    self.child_frame_id = self.declare_parameter("child_frame_id","base_footprint").value

    #set up variable initial value
    self.x = 0.0
    self.y = 0.0
    self.th = 0.0


  def wheel_velocity_callback(self,msg):
    vl = msg.left_wheel
    vr = msg.right_wheel
    #vl = 0.1
    #vr = 0.1

    vx = (self.R / 2) * (vl + vr)         # calculate robot velocity on x direction. unit in m/s
    vy = 0.0                              # Velocity on x direction is zero because this is non-holonomic
    vth = (self.R / self.L) * (vr - vl)   # calculate robot angular velocity z axis. unit in rad/s
                                          # reference @ http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html

    #rospy.loginfo('delta left tick = %d & current left tick = %d', self.d_left_tick, encoder.left_tick)
    #rospy.loginfo('delta right tick = %d & current right tick = %d', self.d_right_tick, encoder.right_tick)
    #rospy.loginfo('delta time = %f', self.dt)

    # compute odometry in a typical way given the velocities of the robot
    delta_x = (vx * cos(self.th) - vy * sin(self.th)) * self.dt
    delta_y = (vx * sin(self.th) + vy * cos(self.th)) * self.dt
    delta_th = vth * self.dt

    self.x += delta_x
    self.y += delta_y
    self.th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    # publish the odom information
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = sin(self.th / 2)
    quaternion.w = cos(self.th / 2)

    transform_stamped_msg = TransformStamped()
    transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
    transform_stamped_msg.header.frame_id = self.frame_id
    transform_stamped_msg.child_frame_id = self.child_frame_id
    transform_stamped_msg.transform.translation.x = self.x
    transform_stamped_msg.transform.translation.y = self.y
    transform_stamped_msg.transform.translation.z = 0.0
    transform_stamped_msg.transform.rotation.x = quaternion.x
    transform_stamped_msg.transform.rotation.y = quaternion.y
    transform_stamped_msg.transform.rotation.z = quaternion.z
    transform_stamped_msg.transform.rotation.w = quaternion.w

    self.odom_broadcaster.sendTransform(transform_stamped_msg)

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = self.get_clock().now().to_msg()
    odom.header.frame_id = self.frame_id
    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = quaternion
    odom.child_frame_id = self.child_frame_id
    odom.twist.twist.linear.x = delta_x
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.angular.z = delta_th
    self.odom_pub.publish(odom)

def main(args=None):
  rclpy.init(args=args)
  try:
      node = Odom()
      rclpy.spin(node)
  except rclpy.exceptions.ROSInterruptException:
      pass

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main(); 


