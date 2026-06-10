#!/usr/bin/env python3
"""ROS 2 (rclpy) node wrapping the MuJoCo quadrotor simulation.

Parameters:
    model_path (str, required) : absolute path to the MuJoCo MJCF file
    hover_thrust (float)       : per-motor hover thrust offset
    max_thrust (float)         : per-motor thrust clip
    init_x / init_y / init_z   : spawn position [m]
    frame_id / child_frame_id  : TF frames for the published odometry

Subscribes:
    target (geometry_msgs/Point)  desired world-frame setpoint [m]

Publishes:
    odom   (nav_msgs/Odometry)    drone state, at the simulation rate
    TF     frame_id -> child_frame_id
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from pairs_softuav_simulation.controller import QuadrotorSim


class DroneSimNode(Node):
    def __init__(self):
        super().__init__("drone_sim_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("hover_thrust", 2.3789)
        self.declare_parameter("max_thrust", 8.0)
        self.declare_parameter("init_x", 0.0)
        self.declare_parameter("init_y", 0.0)
        self.declare_parameter("init_z", 1.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        model_path = self.get_parameter("model_path").value
        if not model_path:
            raise RuntimeError("parameter 'model_path' is required")
        x0 = self.get_parameter("init_x").value
        y0 = self.get_parameter("init_y").value
        z0 = self.get_parameter("init_z").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        self.sim = QuadrotorSim(
            model_path,
            target=(x0, y0, z0),
            hover_thrust=self.get_parameter("hover_thrust").value,
            max_thrust=self.get_parameter("max_thrust").value,
        )
        self.sim.reset_pose(x0, y0, z0)

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Point, "target", self._on_target, 1)
        self.create_timer(self.sim.timestep, self._on_step)

        self.get_logger().info(
            f"drone_sim_node up: model={model_path}, rate={1.0 / self.sim.timestep:.0f} Hz")

    def _on_target(self, msg):
        self.sim.set_target(msg.x, msg.y, msg.z)

    def _on_step(self):
        self.sim.step()
        state = self.sim.get_state()
        stamp = self.get_clock().now().to_msg()
        self._publish_odom(state, stamp)
        self._publish_tf(state, stamp)

    def _publish_odom(self, state, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        p, q = state["position"], state["orientation"]
        v, w = state["linear_velocity"], state["angular_velocity"]
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = \
            float(p[0]), float(p[1]), float(p[2])
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = \
            float(v[0]), float(v[1]), float(v[2])
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = \
            float(w[0]), float(w[1]), float(w[2])
        self.odom_pub.publish(odom)

    def _publish_tf(self, state, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id
        p, q = state["position"], state["orientation"]
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
            float(p[0]), float(p[1]), float(p[2])
        tf.transform.rotation.w = float(q[0])
        tf.transform.rotation.x = float(q[1])
        tf.transform.rotation.y = float(q[2])
        tf.transform.rotation.z = float(q[3])
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = DroneSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
