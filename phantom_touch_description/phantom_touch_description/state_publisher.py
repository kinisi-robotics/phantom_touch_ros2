from math import sin, cos, pi, atan2, asin

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


def deg_to_rad(degree):
    """
    Returns a radian from a deg
    """
    pi = 22 / 7
    return degree * (pi / 180)


class StatePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__("state_publisher")

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        pi / 180.0
        loop_rate = self.create_rate(30)

        joints_names = []
        for i in range(1, 8):
            joints_names.append(f"arm_joint_{i}")

        joints_names.append("gripper_left_finger_joint")
        joints_names.append("gripper_right_finger_joint")

        joints_values = [0.0] * len(joints_names)

        self.get_logger().info(f"Will Publish values for these joints: {joints_names}")

        # robot state
        # If you want a default position
        arm_joint_blank = 0.0

        angle = 0.0

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "axis"
        joint_state = JointState()

        # Used for making temp tf links for calculating rpy for urdf joints
        # self.static_broadcaster(x=0.04,y=0.21651,z=(0.275-0.150),yaw=-0.18273597,base_frame="arm_base", frame="shoulder")
        # self.static_broadcaster(yaw=deg_to_rad(-10.47),base_frame="arm_base", frame="temp1") # 0.18273597
        # self.static_broadcaster(roll=deg_to_rad(30.0),base_frame="temp1", frame="temp2") # 2.61799

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = joints_names
                joint_state.position = joints_values

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle) * 2
                odom_trans.transform.translation.y = sin(angle) * 2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = euler_to_quaternion(
                    0, 0, angle + pi / 2
                )  # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                # self.broadcaster.sendTransform(odom_trans)

                arm_joint_blank += 0.01

                if arm_joint_blank > 1.4:
                    arm_joint_blank = -1.4

                # joints_values[6] = arm_joint_blank
                # joints_values[5] = arm_joint_blank
                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

    def static_broadcaster(
        self, base_frame, frame, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0
    ):
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = base_frame
        t.child_frame_id = frame

        quat = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        self.tf_static_broadcaster.sendTransform(t)


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)
    return roll_x, pitch_y, yaw_z


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(
        pitch / 2
    ) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(
        pitch / 2
    ) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    StatePublisher()


if __name__ == "__main__":
    main()
