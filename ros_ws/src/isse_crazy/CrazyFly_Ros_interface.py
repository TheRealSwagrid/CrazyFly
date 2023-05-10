#!/usr/bin/env python
import rospy
from AbstractVirtualCapability import VirtualCapabilityServer
from CrazyFly import CrazyFly


class CrazyFly_Ros_interface:

    def __init__(self):
        self.target = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.arming_status = False

    def fly_to(self, pos: list):
        self.target = pos

    def arm(self):
        self.arming_status = True

    def disarm(self):
        self.arming_status = False

    def get_position(self):
        self.position

    def get_arming_status(self):
        return self.arming_status


if __name__ == '__main__':
    rospy.init_node('rosnode')
    rate = rospy.Rate(20)

    rospy.logwarn("Starting CrazyFly ROS")
    drone = CrazyFly_Ros_interface()

    rospy.logwarn("Starting server")


    """
    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))
    
    rospy.logwarn("starting isse_copter semanticplugandplay")
    copter = CrazyFly(server)

    copter.functionality["arm"] = drone.arm
    copter.functionality["disarm"] = drone.disarm
    copter.functionality["SetISSECopterPosition"] = drone.fly_to
    copter.functionality["GetISSECopterPosition"] = drone.get_position
    copter.functionality["GetArmingStatus"] = drone.get_arming_status
    copter.start()
    # signal.signal(signal.SIGTERM, handler)
    """

    while not rospy.is_shutdown():# and server.running:
        rate.sleep()
        # rospy.logwarn(f"Server status: {server.running}, {copter}")
