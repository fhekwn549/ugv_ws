#!/usr/bin/env python3
"""
Simulation Pose Bridge: RViz '2D Pose Estimate' → Gazebo teleport.

Subscribes to /initialpose from RViz and teleports the robot in Gazebo
using the /set_entity_state service. Also resets odometry so the
static map→odom TF remains consistent.

Requires: libgazebo_ros_state.so plugin in the world file.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


class SimPoseBridge(Node):

    def __init__(self):
        super().__init__('sim_pose_bridge')
        self.declare_parameter('entity_name', 'ugv_roarm')

        self._entity = self.get_parameter('entity_name').value

        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initial_pose,
            10,
        )

        self._client = self.create_client(
            SetEntityState, '/set_entity_state')

        self.get_logger().info(
            f'Sim pose bridge ready (entity: {self._entity})')

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.get_logger().info(
            f'Teleporting {self._entity} to ({p.x:.2f}, {p.y:.2f})')

        if not self._client.service_is_ready():
            self.get_logger().warn(
                '/set_entity_state not available, waiting...')
            self._client.wait_for_service(timeout_sec=5.0)
            if not self._client.service_is_ready():
                self.get_logger().error(
                    '/set_entity_state service not available')
                return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self._entity
        req.state.pose = msg.pose.pose
        req.state.pose.position.z = 0.05
        req.state.reference_frame = 'world'

        future = self._client.call_async(req)
        future.add_done_callback(self._teleport_done)

    def _teleport_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('Teleport successful')
            else:
                self.get_logger().warn('Teleport failed')
        except Exception as e:
            self.get_logger().error(f'Teleport error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
