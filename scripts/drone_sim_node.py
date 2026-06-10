#!/usr/bin/env python3
"""ROS 1 (rospy) node wrapping the MuJoCo quadrotor simulation.

Parameters (private, ``~``):
    model_path (str, required) : absolute path to the MuJoCo MJCF file
    hover_thrust (float)       : per-motor hover thrust offset
    max_thrust (float)         : per-motor thrust clip
    init_x / init_y / init_z   : spawn position [m]
    frame_id / child_frame_id  : TF frames for the published odometry

Subscribes:
    ~target (geometry_msgs/Point)  desired world-frame setpoint [m]

Publishes:
    ~odom   (nav_msgs/Odometry)    drone state, at the simulation rate
    TF      frame_id -> child_frame_id
"""
import rospy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

from pairs_softuav_sim.controller import QuadrotorSim


class DroneSimNode:
    def __init__(self):
        model_path = rospy.get_param("~model_path")
        x0 = rospy.get_param("~init_x", 0.0)
        y0 = rospy.get_param("~init_y", 0.0)
        z0 = rospy.get_param("~init_z", 1.0)
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        self.sim = QuadrotorSim(
            model_path,
            target=(x0, y0, z0),
            hover_thrust=rospy.get_param("~hover_thrust", 2.3789),
            max_thrust=rospy.get_param("~max_thrust", 8.0),
        )
        self.sim.reset_pose(x0, y0, z0)

        self.odom_pub = rospy.Publisher("~odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("~target", Point, self._on_target, queue_size=1)

        rospy.loginfo("drone_sim_node up: model=%s, rate=%.0f Hz",
                      model_path, 1.0 / self.sim.timestep)
        self.timer = rospy.Timer(rospy.Duration(self.sim.timestep), self._on_step)

    def _on_target(self, msg):
        self.sim.set_target(msg.x, msg.y, msg.z)

    def _on_step(self, _event):
        self.sim.step()
        state = self.sim.get_state()
        now = rospy.Time.now()
        self._publish_odom(state, now)
        self._publish_tf(state, now)

    def _publish_odom(self, state, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        p, q = state["position"], state["orientation"]
        v, w = state["linear_velocity"], state["angular_velocity"]
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = p
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = v
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

    def _publish_tf(self, state, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id
        p, q = state["position"], state["orientation"]
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p
        tf.transform.rotation.w = q[0]
        tf.transform.rotation.x = q[1]
        tf.transform.rotation.y = q[2]
        tf.transform.rotation.z = q[3]
        self.tf_broadcaster.sendTransform(tf)


if __name__ == "__main__":
    rospy.init_node("drone_sim_node")
    DroneSimNode()
    rospy.spin()
