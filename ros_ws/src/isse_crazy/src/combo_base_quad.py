from abc import ABC, abstractmethod

from combo_base_movable import ComboBaseMovable
import json

from isse_crazy.srv import NoArguments, NoArgumentsResponse
from isse_crazy.srv import SetBatteryPercentage, SetBatteryPercentageResponse
import rospy


class ComboBaseQuad(ComboBaseMovable, ABC):

    @abstractmethod
    def set_marker_color(self):
        pass

    def get_marker_resource(self) -> str:
        return "package://isse_crazy/meshes/crazyflie/crazyflie.dae"

    def __init__(self, is_sim):
        super().__init__(is_sim=is_sim)

        self.set_marker_color()

        self.move_to_charge_srv = rospy.Service("move_to_charge", NoArguments, self.move_to_charge)
        self.land_srv = rospy.Service("land", NoArguments, self.land)
        self.state_srv = rospy.Service("state", NoArguments, self.handle_state)

    def publish_state(self):
        data = {'batteryPercentage': self.current_battery_percentage}
        self.state_pub.publish(json.dumps(data))

    def move_to_charge(self, req):
        """
        handler for service move_to_charge
        own service to easily adjust this logic in future
        """
        self.cf.land(0.1, 3)
        ret = NoArgumentsResponse()
        ret.status = 1
        return ret

    def land(self, req):
        """
        handler for service land
        """
        self.cf.land(0.1, 3)
        ret = NoArgumentsResponse()
        ret.status = 1
        return ret

    def handle_state(self, req):
        data = {'batteryPercentage': self.current_battery_percentage}
        return json.dumps(data)

    def handle_set_battery_percentage(self, req):
        """
        handler for set_bat_prc
        Used to simulate low battery from command line
        No further usage needed!
        """
        ret = SetBatteryPercentageResponse()
        ret.status = 1
        self.current_battery_percentage = req.percentage.data
        return ret
