#!/usr/bin/env python3
"""ROS 2 front-end for the Gazebo Harmonic DetachableJoint link attacher.

Keeps the same interface the Classic IFRA link attacher had:

    /ATTACHLINK  (linkattacher_msgs/srv/AttachLink)
    /DETACHLINK  (linkattacher_msgs/srv/DetachLink)

so pick_place_gazebo.cpp keeps working unchanged.  Internally each request is
turned into an std_msgs/Empty on
    /gz_link_attacher/<model2_name>/attach   or   .../detach
which ros_gz_bridge forwards to the DetachableJoint plugin that
panda.gz_attachers.xacro put into the robot description for that piece.

The plugins start in the *attached* state as soon as the piece is spawned, so
this node also sends a "detach" to every piece once a second for the first
`startup_detach_seconds` (until the piece has been explicitly attached).
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from linkattacher_msgs.srv import AttachLink, DetachLink
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

TOPIC_PREFIX = '/gz_link_attacher'


def load_pieces(layout_file=''):
  if not layout_file:
    layout_file = os.path.join(
        get_package_share_directory('moveit_config_pkg'),
        'config', 'initial_board_layout.json')
  with open(layout_file, 'r') as f:
    return sorted(set(json.load(f).values()))


class GzLinkAttacher(Node):

  def __init__(self):
    super().__init__('gz_link_attacher')
    self.declare_parameter('layout_file', '')
    self.declare_parameter('startup_detach_seconds', 90.0)
    self.declare_parameter('parent_link', 'panda_leftfinger')

    self.parent_link = self.get_parameter('parent_link').value
    self.pieces = load_pieces(self.get_parameter('layout_file').value)
    self.attached = {p: False for p in self.pieces}
    self.attach_pubs = {
        p: self.create_publisher(Empty, f'{TOPIC_PREFIX}/{p}/attach', 10)
        for p in self.pieces
    }
    self.detach_pubs = {
        p: self.create_publisher(Empty, f'{TOPIC_PREFIX}/{p}/detach', 10)
        for p in self.pieces
    }

    self.create_service(AttachLink, '/ATTACHLINK', self.on_attach)
    self.create_service(DetachLink, '/DETACHLINK', self.on_detach)

    self.startup_deadline = (
        self.get_clock().now().nanoseconds * 1e-9
        + float(self.get_parameter('startup_detach_seconds').value))
    self.create_timer(1.0, self.startup_detach)
    self.get_logger().info(
        f'gz_link_attacher ready for {len(self.pieces)} pieces '
        f'(parent link: {self.parent_link}).')

  def startup_detach(self):
    now = self.get_clock().now().nanoseconds * 1e-9
    if now > self.startup_deadline:
      return
    for p in self.pieces:
      if not self.attached[p]:
        self.detach_pubs[p].publish(Empty())

  def _check(self, req):
    if req.model2_name not in self.attach_pubs:
      return (f'No DetachableJoint configured for model "{req.model2_name}". '
              'Is it listed in initial_board_layout.json?')
    if req.link1_name != self.parent_link:
      self.get_logger().warn(
          f'link1_name "{req.link1_name}" ignored; the joint is bound to '
          f'"{self.parent_link}" in panda.gz_attachers.xacro.')
    return None

  def on_attach(self, req, res):
    err = self._check(req)
    if err:
      res.success, res.message = False, err
      return res
    self.attach_pubs[req.model2_name].publish(Empty())
    self.attached[req.model2_name] = True
    res.success, res.message = True, f'attach requested for {req.model2_name}'
    self.get_logger().info(res.message)
    return res

  def on_detach(self, req, res):
    err = self._check(req)
    if err:
      res.success, res.message = False, err
      return res
    self.detach_pubs[req.model2_name].publish(Empty())
    self.attached[req.model2_name] = False
    res.success, res.message = True, f'detach requested for {req.model2_name}'
    self.get_logger().info(res.message)
    return res


def main():
  rclpy.init()
  node = GzLinkAttacher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  node.destroy_node()
  if rclpy.ok():
    rclpy.shutdown()


if __name__ == '__main__':
  main()
