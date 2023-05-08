#!/usr/bin/env python3

from abc import ABC, abstractmethod

import rospy

import geometry_msgs.msg
import visualization_msgs.msg
import std_msgs.msg
from isse_crazy.msg import PoseVelStamped

from isse_crazy.srv import Position, PositionResponse

import tf2_ros


class ComboBase:
    """
    This is the main object for classes like simulated quad, crazyflie, simulated wand and real wand
    """

    @abstractmethod
    def init_node(self):
        pass

    @abstractmethod
    def init_cf(self):
        pass

    @abstractmethod
    def execute_in_loop(self):
        pass

    @abstractmethod
    def init_additional_publishers(self):
        pass

    @abstractmethod
    def get_marker_resource(self) -> str:
        pass

    @abstractmethod
    def get_position(self) -> [float]:
        pass

    def __set_marker_values(self):
        self.marker.header.frame_id = self.string_id
        # TODO: maybe use a shared namespace for all fake quads and another one for all real quads.
        #       Then it's easier to disable for example all fake quads with one click in RViz.

        self.marker.ns = self.string_id
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.mesh_resource = self.get_marker_resource()
        self.marker.mesh_use_embedded_materials = True
        self.marker.id = 0  # NOTE: if multiple quads share a namespace, the ids have to differ
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.pose.orientation.w = 1.0

    def __init__(self):
        self.init_node()
        self.id = rospy.get_param("~id")
        self.string_id = str(self.id)

        # init fields
        self.position_pub = rospy.Publisher("position", geometry_msgs.msg.Pose, queue_size=100)
        self.pose_vel_stamped_pub = rospy.Publisher("pose_vel_stamped", PoseVelStamped, queue_size=100)
        self.velocity_pub = rospy.Publisher("velocity", geometry_msgs.msg.Pose, queue_size=100)

        # init rate
        self.hz = 20
        self.rate = rospy.Rate(self.hz)

        # used for fakeflie
        self.timeHelper = None

        # init cf
        self.cf = None
        self.init_cf()

        self.lastPos = [0, 0, 0]
        self.currentPos = [0, 0, 0]

        # init marker object
        self.marker = visualization_msgs.msg.Marker()
        self.__set_marker_values()

        self.position_srv = rospy.Service("position", Position, self.get_pos)

        self.marker_pub = rospy.Publisher("/visualization_marker", visualization_msgs.msg.Marker, queue_size=2)
        self.broadcaster = tf2_ros.TransformBroadcaster()

    def publish_marker(self):
        self.marker_pub.publish(self.marker)

    def publish_position(self):
        pose = geometry_msgs.msg.Pose()
        pos = self.get_position()

        # set last pos for velocity
        self.lastPos = self.currentPos
        self.currentPos = pos

        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        self.position_pub.publish(pose)

    def get_pos(self, request):
        """
        handler for service get pos
        """
        pos = self.cf.position()
        ret = PositionResponse()
        ret.pose = geometry_msgs.msg.Pose()
        ret.pose.position.x = pos[0]
        ret.pose.position.y = pos[1]
        ret.pose.position.z = pos[2]
        return ret

    def publish_velocity(self):
        vel = geometry_msgs.msg.Pose()

        calculated_velocity = self.__calculate_velocity()
        vel.position.x = calculated_velocity[0]
        vel.position.y = calculated_velocity[1]
        vel.position.z = calculated_velocity[2]
        self.velocity_pub.publish(vel)

    def __calculate_velocity(self) -> [float]:
        vel = [0, 0, 0]
        vel[0] = (self.lastPos[0] - self.currentPos[0]) * self.hz
        vel[1] = (self.lastPos[1] - self.currentPos[1]) * self.hz
        vel[2] = (self.lastPos[2] - self.currentPos[2]) * self.hz
        return vel

    def publish_pose_vel_stamped(self):
        message = PoseVelStamped()

        pose = geometry_msgs.msg.Pose()
        vel = geometry_msgs.msg.Twist()

        current_pose = self.get_position()
        pose.position.x = current_pose[0]
        pose.position.y = current_pose[1]
        pose.position.z = current_pose[2]

        calculated_velocity = self.__calculate_velocity()
        vel.linear.x = calculated_velocity[0]
        vel.linear.y = calculated_velocity[1]
        vel.linear.z = calculated_velocity[2]

        message.pose = pose
        message.velocity = vel
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.string_id
        self.pose_vel_stamped_pub.publish(message)
