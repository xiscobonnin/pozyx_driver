#!/usr/bin/env python3

import ast
import math

import rclpy
from rclpy.node import Node

import pypozyx
from pypozyx import (
    POZYX_3D,
    POZYX_ANCHOR_SEL_AUTO,
    POZYX_POS_ALG_TRACKING,
    POZYX_SUCCESS,
    AngularVelocity,
    Coordinates,
    DeviceCoordinates,
    DeviceList,
    DeviceRange,
    LinearAcceleration,
    PozyxSerial,
    PositionError,
    Pressure,
    Quaternion as PozyxQuaternion,
    SingleRegister,
)

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion, Vector3
from uwb_msgs.msg import AnchorInfo
from sensor_msgs.msg import FluidPressure, Imu


class PozyxDriverNode(Node):
    def __init__(self):
        super().__init__('pozyx_node')

        self._declare_parameters()
        self._load_parameters()

        self.pose_with_cov_pub = self.create_publisher(PoseWithCovarianceStamped, 'pose_with_cov', 1)
        self.imu_pub = self.create_publisher(Imu, 'imu', 1)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 1)
        self.pressure_pub = self.create_publisher(FluidPressure, 'pressure', 1)
        self.anchor_info_pubs = [
            self.create_publisher(AnchorInfo, f'anchor_info_{i}', 1)
            for i in range(len(self.anchors))
        ]

        self.pozyx = PozyxSerial(self.serial_port)
        self.range_error_counts = [0 for _ in range(len(self.anchors))]

        if not self.set_anchors_manual():
            self.get_logger().warning('Anchor configuration reported errors.')
        self.print_publish_configuration_result()

        period = 1.0 / float(self.frequency)
        self.timer = self.create_timer(period, self.loop)
        self.get_logger().info(
            f'Pozyx driver started on {self.serial_port} at {self.frequency} Hz with {len(self.anchors)} anchors.'
        )

    def _declare_parameters(self):
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('num_anchors', 4)
        self.declare_parameter('tag_device_id', 'None')
        self.declare_parameter('algorithm', int(POZYX_POS_ALG_TRACKING))
        self.declare_parameter('dimension', int(POZYX_3D))
        self.declare_parameter('height', 1000)
        self.declare_parameter('frequency', 15)
        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('tag_frame_id', 'pozyx_tag')
        self.declare_parameter('do_ranging_attempts', 1)

    def _load_parameters(self):
        self.serial_port = self.get_parameter('serial_port').value
        self.num_anchors = int(self.get_parameter('num_anchors').value)
        self.tag_device_id = self._parse_optional_int(self.get_parameter('tag_device_id').value)
        self.algorithm = int(self.get_parameter('algorithm').value)
        self.dimension = int(self.get_parameter('dimension').value)
        self.height = int(self.get_parameter('height').value)
        self.frequency = max(1, int(self.get_parameter('frequency').value))
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.tag_frame_id = self.get_parameter('tag_frame_id').value
        self.do_ranging_attempts = max(1, int(self.get_parameter('do_ranging_attempts').value))

        self.anchors = []
        for i in range(self.num_anchors):
            id_name = f'anchor{i}_id'
            coord_name = f'anchor{i}_coordinates'
            self.declare_parameter(id_name, '')
            self.declare_parameter(coord_name, '[0, 0, 0]')

            network_id = self._parse_hex_or_int(self.get_parameter(id_name).value)
            coords = self._parse_coordinates(self.get_parameter(coord_name).value)
            self.anchors.append(
                DeviceCoordinates(
                    network_id,
                    1,
                    Coordinates(coords[0], coords[1], coords[2])
                )
            )

    def _parse_optional_int(self, value):
        if value is None:
            return None
        if isinstance(value, int):
            return value
        text = str(value).strip()
        if text.lower() in ('none', ''):
            return None
        return self._parse_hex_or_int(text)

    def _parse_hex_or_int(self, value):
        if isinstance(value, int):
            return value
        text = str(value).strip()
        return int(text, 0)

    def _parse_coordinates(self, value):
        if isinstance(value, (list, tuple)) and len(value) == 3:
            return [int(value[0]), int(value[1]), int(value[2])]
        parsed = ast.literal_eval(str(value))
        if not isinstance(parsed, (list, tuple)) or len(parsed) != 3:
            raise ValueError(f'Invalid anchor coordinates: {value}')
        return [int(parsed[0]), int(parsed[1]), int(parsed[2])]

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _fill_quaternion(self, ros_quat_msg, pozyx_quat):
        ros_quat_msg.x = float(pozyx_quat.x)
        ros_quat_msg.y = float(pozyx_quat.y)
        ros_quat_msg.z = float(pozyx_quat.z)
        ros_quat_msg.w = float(pozyx_quat.w)

    def _fill_point_mm_to_m(self, ros_point_msg, pozyx_coords):
        ros_point_msg.x = float(pozyx_coords.x) * 0.001
        ros_point_msg.y = float(pozyx_coords.y) * 0.001
        ros_point_msg.z = float(pozyx_coords.z) * 0.001

    def _fill_vector3(self, ros_vector_msg, x, y, z):
        ros_vector_msg.x = float(x)
        ros_vector_msg.y = float(y)
        ros_vector_msg.z = float(z)

    def loop(self):
        pose_msg = self._build_pose_with_covariance()
        if pose_msg is not None:
            self.pose_with_cov_pub.publish(pose_msg)
            self.pose_pub.publish(self._build_pose_stamped(pose_msg))

        self.imu_pub.publish(self._build_imu())
        self._publish_anchor_info()
        self.pressure_pub.publish(self._build_pressure())

    def _build_pose_with_covariance(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.world_frame_id

        position = Coordinates()
        orientation = PozyxQuaternion()
        cov = PositionError()

        status = self.pozyx.doPositioning(
            position,
            self.dimension,
            self.height,
            self.algorithm,
            self.tag_device_id,
        )
        self.pozyx.getQuaternion(orientation, self.tag_device_id)
        self.pozyx.getPositionError(cov, self.tag_device_id)

        self._fill_point_mm_to_m(msg.pose.pose.position, position)
        self._fill_quaternion(msg.pose.pose.orientation, orientation)

        msg.pose.covariance = [
            float(cov.x), float(cov.xy), float(cov.xz), 0.0, 0.0, 0.0,
            float(cov.xy), float(cov.y), float(cov.yz), 0.0, 0.0, 0.0,
            float(cov.xz), float(cov.yz), float(cov.z), 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]

        if status != POZYX_SUCCESS:
            self.get_logger().warning('doPositioning failed; pose was not published.', throttle_duration_sec=5.0)
            return None

        return msg

    def _build_pose_stamped(self, pose_with_cov_msg):
        msg = PoseStamped()
        msg.header.stamp = pose_with_cov_msg.header.stamp
        msg.header.frame_id = pose_with_cov_msg.header.frame_id
        msg.pose = pose_with_cov_msg.pose.pose
        return msg

    def _build_imu(self):
        msg = Imu()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.tag_frame_id
        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9

        orientation = PozyxQuaternion()
        angular_velocity = AngularVelocity()
        linear_acceleration = LinearAcceleration()

        self.pozyx.getQuaternion(orientation, self.tag_device_id)
        self.pozyx.getAngularVelocity_dps(angular_velocity, self.tag_device_id)
        self.pozyx.getLinearAcceleration_mg(linear_acceleration, self.tag_device_id)

        self._fill_quaternion(msg.orientation, orientation)

        self._fill_vector3(
            msg.angular_velocity,
            angular_velocity.x * (math.pi / 180.0),
            angular_velocity.y * (math.pi / 180.0),
            angular_velocity.z * (math.pi / 180.0),
        )
        self._fill_vector3(
            msg.linear_acceleration,
            linear_acceleration.x * 0.0098,
            linear_acceleration.y * 0.0098,
            linear_acceleration.z * 0.0098,
        )

        return msg

    def _publish_anchor_info(self):
        for i, anchor in enumerate(self.anchors):
            msg = AnchorInfo()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = self.world_frame_id
            msg.child_frame_id = f'anchor_{i}'
            msg.id = hex(anchor.network_id)
            msg.position = Point(
                x=float(anchor.pos.x) * 0.001,
                y=float(anchor.pos.y) * 0.001,
                z=float(anchor.pos.z) * 0.001,
            )
            msg.position_cov = [0.0] * 9
            msg.distance_cov = 0.0

            iter_ranging = 0
            while iter_ranging < self.do_ranging_attempts:
                device_range = DeviceRange()
                status = self.pozyx.doRanging(anchor.network_id, device_range, self.tag_device_id)
                msg.distance = float(device_range.distance) * 0.001
                msg.rss = int(device_range.RSS)

                if status == POZYX_SUCCESS:
                    msg.status = True
                    self.range_error_counts[i] = 0
                    break

                msg.status = False
                self.range_error_counts[i] += 1
                if self.range_error_counts[i] > 9:
                    self.range_error_counts[i] = 0
                    self.get_logger().error(f'Anchor {i} ({msg.id}) lost')
                iter_ranging += 1

            self.anchor_info_pubs[i].publish(msg)

    def _build_pressure(self):
        msg = FluidPressure()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.tag_frame_id

        pressure = Pressure()
        self.pozyx.getPressure_Pa(pressure, self.tag_device_id)
        msg.fluid_pressure = float(pressure.value)
        msg.variance = 0.0
        return msg

    def set_anchors_manual(self):
        status = self.pozyx.clearDevices()
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors))
        return status == POZYX_SUCCESS

    def print_publish_configuration_result(self):
        list_size = SingleRegister()
        status = self.pozyx.getDeviceListSize(list_size)
        if status != POZYX_SUCCESS:
            self.get_logger().warning('Unable to read anchor list size from Pozyx device.')
            return

        device_list = DeviceList(list_size=list_size[0])
        status = self.pozyx.getDeviceIds(device_list)
        if status != POZYX_SUCCESS:
            self.get_logger().warning('Unable to read anchor IDs from Pozyx device.')
            return

        self.get_logger().info('Anchors configuration:')
        self.get_logger().info(f'Anchors found: {list_size[0]}')
        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            status = self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates)
            if status == POZYX_SUCCESS:
                self.get_logger().info(f'ANCHOR,0x{device_list[i]:0.4x}, {anchor_coordinates}')
            else:
                self.get_logger().warning(f'Unable to read coordinates for anchor 0x{device_list[i]:0.4x}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PozyxDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
