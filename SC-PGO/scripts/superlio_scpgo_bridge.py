#!/usr/bin/env python3
"""Convert synchronized Super-LIO world clouds into the body clouds PGO expects."""
from collections import OrderedDict
from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


def world_to_body(cloud, odom):
    if cloud.header.frame_id != odom.header.frame_id:
        raise ValueError('cloud and odometry must use the same world frame')
    fields = {f.name: f for f in cloud.fields}
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32 or
           fields[name].count != 1 for name in ('x', 'y', 'z')):
        raise ValueError('cloud must contain scalar FLOAT32 x/y/z fields')
    q = odom.pose.pose.orientation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid odometry quaternion')
    x, y, z, w = quat / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    p = odom.pose.pose.position
    translation = np.array([p.x, p.y, p.z])
    if not np.isfinite(translation).all():
        raise ValueError('invalid odometry position')
    data = bytearray(cloud.data)
    dtype = '>f4' if cloud.is_bigendian else '<f4'
    coordinates = [np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=data,
                             offset=fields[name].offset,
                             strides=(cloud.row_step, cloud.point_step))
                   for name in ('x', 'y', 'z')]
    xyz = np.stack(coordinates, axis=-1).astype(np.float64)
    body = (xyz - translation) @ rotation
    for axis, coordinate in enumerate(coordinates):
        coordinate[:] = body[..., axis]
    result = deepcopy(cloud)
    result.header.frame_id = 'super_lio_body'
    result.data = bytes(data)
    return result


class SuperLIOBridge(Node):
    def __init__(self):
        super().__init__('superlio_scpgo_bridge')
        self.clouds = OrderedDict()
        self.odoms = OrderedDict()
        self.cloud_pub = self.create_publisher(PointCloud2, '/superlio/scpgo/cloud_body', 20)
        self.odom_pub = self.create_publisher(Odometry, '/superlio/scpgo/odom', 20)
        self.create_subscription(PointCloud2, '/lio/cloud_world',
                                 lambda msg: self.receive(msg, self.clouds), 20)
        self.create_subscription(Odometry, '/lio/odom',
                                 lambda msg: self.receive(msg, self.odoms), 20)

    def receive(self, msg, buffer):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        buffer[key] = msg
        if key in self.clouds and key in self.odoms:
            cloud, odom = self.clouds.pop(key), self.odoms.pop(key)
            try:
                local_cloud = world_to_body(cloud, odom)
            except (ValueError, TypeError, BufferError) as error:
                self.get_logger().warning(f'Dropping invalid synchronized pair: {error}')
                return
            local_odom = deepcopy(odom)
            # camera_init names the same initial world origin in the PGO pipeline.
            local_odom.header.frame_id = 'camera_init'
            local_odom.child_frame_id = 'super_lio_body'
            self.cloud_pub.publish(local_cloud)
            self.odom_pub.publish(local_odom)
        while len(buffer) > 100:
            buffer.popitem(last=False)


def main():
    rclpy.init()
    node = SuperLIOBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
