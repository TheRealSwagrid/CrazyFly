#!/usr/bin/env python
import rospy
from AbstractVirtualCapability import VirtualCapabilityServer
from CrazyFly import CrazyFly

from tf import TransformListener
from pycrazyswarm.crazyflie import CrazyflieServer, Crazyflie

class CrazyFly_Ros_interface:

    def __init__(self):
        self.target = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.arming_status = False
        self.id = int(rospy.get_param('~cf_id'))
        self.transformListener = TransformListener()
        for crazyflie in rospy.get_param("/crazyflies"):
            rospy.logwarn(crazyflie["id"])
            rospy.logwarn(str(self.id) + "\n")
            if int(crazyflie["id"]) == self.id:
                self.cf = Crazyflie(self.id, crazyflie["initialPosition"], self.transformListener)
                break


    def fly_to(self, pos: list):
        self.cf.goTo(pos, 0, 5)

    def arm(self):
        self.arming_status = True
        self.cf.setLEDColor(0, 255, 0)
        self.cf.takeoff(1.0, 3.0)
        rospy.sleep(3)

    def disarm(self):
        self.arming_status = False
        self.cf.setLEDColor(255, 0, 0)
        self.cf.land(0., 3.0)
        rospy.sleep(3)

    def get_position(self):
        return self.cf.position()

    def get_arming_status(self):
        return self.arming_status

    def select_cf(self, cf_id: int):
        for crazyflie in rospy.get_param("/crazyflies"):
            rospy.logwarn(crazyflie["id"])
            rospy.logwarn(str(self.id) + "\n")
            if int(crazyflie["id"]) == self.id:
                self.cf = Crazyflie(self.id, crazyflie["initialPosition"], self.transformListener)
                break

    def hover(self):
        self.cf.takeoff(1.0, 3.0)
        rospy.sleep(3)
        self.cf.land(0., 3.0)
        rospy.sleep(3)


if __name__ == '__main__':
    rospy.init_node('rosnode')
    rate = rospy.Rate(20)

    rospy.logwarn("Starting CrazyFly ROS")
    drone = CrazyFly_Ros_interface()

    rospy.logwarn("Starting server")


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


    while not rospy.is_shutdown() and server.running:
        rate.sleep()
        # rospy.logwarn(f"Server status: {server.running}, {copter}")
