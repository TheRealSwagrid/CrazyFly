#!/usr/bin/env python3
from abc import ABC, abstractmethod, ABCMeta

import time
import json

from isse_crazy.srv import MoveTo, MoveToResponse
from isse_crazy.srv import Position, PositionResponse
from isse_crazy.srv import State, StateResponse
from isse_crazy.srv import MoveVel, MoveVelResponse
from isse_crazy.srv import NoArguments, NoArgumentsResponse

from tf.transformations import quaternion_from_euler

from combo_base import ComboBase

import geometry_msgs.msg

import rospy


class ComboBaseMovable(ComboBase, metaclass=ABCMeta):
    def init_additional_publishers(self):
        pass

    def execute_in_loop(self):
        # self.publish_state()
        self.publish_position()
        self.publish_marker()
        self.publish_velocity()
        self.publish_pose_vel_stamped()

    def __init__(self, is_sim):
        super().__init__()
        self.is_sim = is_sim

        # services for flying
        self.move_pos_srv = rospy.Service("move_pos", MoveTo, self.move_pos)
        self.cmd_pos_srv = rospy.Service("cmd_pos", MoveTo, self.cmd_pos)
        self.move_vel_srv = rospy.Service("move_vel", MoveVel, self.move_vel)

        self.current_battery_percentage = 1.0

    def move_pos(self, request):
        """
        trying to move to given pose in 4s
        """
        pose = request.pose
        ret = MoveToResponse()
        ret.status = 1
        self.cf.goTo([pose.position.x, pose.position.y, pose.position.z], 0, 4)
        time.sleep(4)
        return ret

    def cmd_pos(self, request):
        """
        cmd pos
        """
        pose = request.pose
        ret = MoveToResponse()
        ret.status = 1
        self.cf.cmdPosition([pose.position.x, pose.position.y, pose.position.z], 0)
        return ret

    def move_vel(self, request):
        velocity = request.velocity
        ret = MoveVelResponse()
        ret.status = 1
        lin = [velocity.linear.x, velocity.linear.y, velocity.linear.z]
        yaw = velocity.angular.z
        self.cf.cmdVelocityWorld(lin, yaw)
        return ret

    def position_to_transform_stamped(self, pos: [], frame: str = "", child_frame: str = "") \
            -> geometry_msgs.msg.TransformStamped:
        """
        converts a position array to a TransformStamped msg with no rotation
        """
        transform = geometry_msgs.msg.TransformStamped()
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
