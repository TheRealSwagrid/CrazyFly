#!/usr/bin/env python3

import time
import geometry_msgs.msg
import nav_msgs.msg
import visualization_msgs.msg
import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from isse_crazy.srv import MoveTo, MoveToResponse
from isse_crazy.srv import Position, PositionResponse
from isse_crazy.srv import State, StateResponse
from isse_crazy.srv import MoveVel, MoveVelResponse
from pycrazyswarm.crazyflieSim import TimeHelper, CrazyflieServer, Crazyflie

from combo_base_movable import ComboBaseMovable


class ComboSimWand(ComboBaseMovable):
    def init_node(self):
        rospy.init_node('combo_sim_wand')

    def get_marker_resource(self) -> str:
        return "package://isse_crazy/meshes/wand.dae"

    def init_cf(self):
        self.timeHelper = TimeHelper('null', 1 / self.hz, False, 0.01)
        self.cf = Crazyflie(self.id, [0, 0, 0], self.timeHelper)

    def get_position(self) -> [float]:
        return self.cf.position()

    def move_pos(self, request):
        # little hack to switch cf.mode to Crazyflie.MODE_HIGH_POLY
        if self.cf.mode != Crazyflie.MODE_HIGH_POLY:
            pos = self.cf.position()
            self.cf.takeoff(self.cf.position()[2], 0)  # switches mode (but also teleports to [0, 0, z]
            self.cf.goTo(pos, 0, 0)  # teleport back to pos
        pose = request.pose
        ret = MoveToResponse()
        ret.status = 1
        self.cf.goTo([pose.position.x, pose.position.y, pose.position.z], 0, 5)
        time.sleep(4)
        return ret

    def __init__(self):
        super().__init__(is_sim=True)

        # init time helper
        self.timeHelper.crazyflies = [self.cf]

        self.broadcaster = tf2_ros.TransformBroadcaster()

        while not rospy.is_shutdown():
            self.timeHelper.step(1 / self.hz)
            # publishes a tf frame
            self.broadcaster.sendTransform(self.position_to_transform_stamped(
                self.cf.position(), "world", self.string_id)
            )
            self.execute_in_loop()
            self.rate.sleep()


if __name__ == "__main__":
    ComboSimWand()
