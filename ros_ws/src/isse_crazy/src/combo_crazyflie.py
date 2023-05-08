#!/usr/bin/env python3
import time

import geometry_msgs.msg
import visualization_msgs.msg
import rospy
from tf import TransformListener

from isse_crazy.srv import MoveTo, MoveToResponse
from isse_crazy.srv import Position, PositionResponse
from isse_crazy.srv import State, StateResponse
from isse_crazy.srv import MoveVel, MoveVelResponse
from pycrazyswarm.crazyflie import CrazyflieServer, Crazyflie
# from cflib.crazyflie.log import LogConfig
from combo_base_quad import ComboBaseQuad


class ComboCrazyflie(ComboBaseQuad):

    def get_position(self) -> [float]:
        return self.cf.position()

    def set_marker_color(self):
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

    def init_cf(self):
        self.transformListener = TransformListener()
        for crazyflie in rospy.get_param("/crazyflies"):
            if crazyflie["id"] == self.id:
                self.cf = Crazyflie(self.id, crazyflie["initialPosition"], self.transformListener)
                break

    def init_node(self):
        rospy.init_node('combo_real_copter')

    def __init__(self):
        super().__init__(is_sim=False)
        self.transformListener = None

        # override fields for crazyflie
        self.hz = 30
        self.rate = rospy.Rate(self.hz)

        while not rospy.is_shutdown():
            self.execute_in_loop()
            self.rate.sleep()


if __name__ == "__main__":
    ComboCrazyflie()
