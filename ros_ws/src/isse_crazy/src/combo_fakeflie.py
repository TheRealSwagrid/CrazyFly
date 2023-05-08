#!/usr/bin/env python3

import time
import rospy
import tf2_ros
import geometry_msgs.msg

from isse_crazy.srv import MoveTo, MoveToResponse

from pycrazyswarm.crazyflieSim import TimeHelper, CrazyflieServer, Crazyflie
from combo_base_quad import ComboBaseQuad


class ComboFakeflie(ComboBaseQuad):
    def init_cf(self):
        self.timeHelper = TimeHelper('null', 1 / self.hz, False, 0.01)
        self.cf = Crazyflie(self.id, [0, 0, 0], self.timeHelper)

    def init_node(self):
        rospy.init_node('combo_fake_copter')

    def get_position(self) -> [float]:
        return self.cf.position()

    def set_marker_color(self):
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.5
        self.marker.color.b = 1.0

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
        super().__init__(is_sim=False)

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
    ComboFakeflie()
