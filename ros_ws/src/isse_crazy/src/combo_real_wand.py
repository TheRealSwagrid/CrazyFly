#!/usr/bin/env python3

import time
import geometry_msgs.msg
import rospy
import tf2_ros
import re

from geometry_msgs.msg import TransformStamped
from isse_crazy.srv import MoveTo, MoveToResponse
from isse_crazy.srv import Position, PositionResponse
from isse_crazy.srv import State, StateResponse
from isse_crazy.srv import MoveVel, MoveVelResponse
from tf.transformations import quaternion_from_euler

from combo_base import ComboBase


def position_to_transform_stamped(pos: [], frame: str = "", child_frame: str = "") -> TransformStamped:
    """
    converts a position array to a TransformStamped msg with no rotation
    """
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = pos[0]
    transform.transform.translation.y = pos[1]
    transform.transform.translation.z = pos[2]
    q = quaternion_from_euler(0, 0, 0)
    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]
    return transform


class ComboRealWand(ComboBase):

    def init_node(self):
        rospy.init_node('combo_real_wand')

    def init_cf(self):
        pass

    def execute_in_loop(self):
        pass

    def init_additional_publishers(self):
        pass

    def get_marker_resource(self) -> str:
        return "package://isse_crazy/meshes/wand.dae"

    def get_position(self) -> [float]:
        return self.vicon_pose

    def __init__(self):
        super().__init__()

        self.vicon_pose_pub_name = "/vrpn_client_node/" + self.id + "/pose"
        self.vicon_sub = rospy.Subscriber(self.vicon_pose_pub_name,
                                          geometry_msgs.msg.PoseStamped,
                                          self.handle_new_vicon_pose
                                          )
        
        # save new vicon pose here to return in get_position
        self.vicon_pose = [0, 0, 0]

        while not rospy.is_shutdown():
            self.publish_position()
            self.publish_velocity()
            self.rate.sleep()

    def handle_new_vicon_pose(self, msg):
        # set last pos for velocity
        self.lastPos = self.currentPos
        self.vicon_pose[0] = msg.pose.position.x
        self.vicon_pose[1] = msg.pose.position.y
        self.vicon_pose[2] = msg.pose.position.z


if __name__ == "__main__":
    ComboRealWand()
