#!/usr/bin/env python3

from typing import Tuple, List, Any

import roslaunch
import rospy
import re

from isse_crazy.srv import StartQuadService, StartQuadServiceResponse
from isse_crazy.srv import GetQuads, GetQuadsResponse
from isse_crazy.srv import GetWands, GetWandsResponse
from isse_crazy.srv import StartWand, StartWandResponse
from isse_crazy.msg import IdObject
from pycrazyswarm.crazyflie import TimeHelper, CrazyflieServer, Crazyflie
from rospy import ROSException
from tf import TransformListener


def get_crazyflies(transformListener) -> Tuple[List[Any], str]:
    """
    returns crazyflie objects for all crazyflies (config in crazyflies.yaml) that have been successfully launched
    or an empty list + error msg if one of the crazyflies can't be reached
    """
    crazyflies = []
    error = None
    try:
        cfg = rospy.get_param("~/crazyflies")
        for crazyflie in cfg:
            cf_id = int(crazyflie["id"])
            initial_position = crazyflie["initialPosition"]
            # check for a random service of this cf to exist, throws timeout if not
            rospy.wait_for_service("/cf" + str(cf_id) + "/set_group_mask", 4)
            cf = Crazyflie(cf_id, initial_position, transformListener)
            crazyflies.append(cf)
    except ROSException as e:
        error = str(e).replace("/set_group_mask", "")
    except Exception as e:
        error = str(e)
    return crazyflies, error


class StartQuadServiceClass:
    def __init__(self):
        rospy.init_node('combo_start_quad_service')

        self.launch_queue = []
        self.launch_parents = []

        self.s1 = rospy.Service('start_real_quad', StartQuadService, self.handle_real)
        self.s2 = rospy.Service('start_sim_quad', StartQuadService, self.handle_fake)
        self.s3 = rospy.Service('get_quads', GetQuads, self.handle_get_quads)
        self.s4 = rospy.Service('start_sim_wand', StartWand, self.handle_sim_wand)
        self.s4 = rospy.Service('start_real_wand', StartWand, self.handle_real_wand)
        self.s5 = rospy.Service('get_wands', GetWands, self.handle_get_wands)

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            for entry in self.launch_queue:
                pkg, file, nr = entry
                roslaunch_file = roslaunch.rlutil.resolve_launch_arguments([pkg, file])[0]
                roslaunch_args = ['cf_id:=' + str(nr)]
                parent = roslaunch.parent.ROSLaunchParent(self.uuid, [(roslaunch_file, roslaunch_args)])
                parent.start()
                self.launch_parents.append(parent)

            self.launch_queue.clear()
            self.rate.sleep()

        for parent in self.launch_parents:
            rospy.logwarn("shutting down " + parent.run_id)
            parent.shutdown()

    def handle_get_quads(self, request) -> GetQuadsResponse:
        """
        handler for /combo/get_quads service
        """
        ret = GetQuadsResponse()
        ret.quads = []
        # Create the IdObject messages from the matching node names
        quads = self.get_quads()
        ret.quads = quads
        return ret

    def handle_get_wands(self, request) -> GetWandsResponse:
        """
        handler for /combo/get_wands service
        """
        ret = GetWandsResponse()
        ret.wands = []
        # Create the IdObject messages from the matching node names
        wands = self.get_wands()
        ret.wands = wands
        return ret

    def handle_real(self, request) -> StartQuadServiceResponse:
        """
        handler for /combo/start_real_quad service
        """
        nr = request.id
        self.launch_queue.append(('isse_crazy', 'real_copter.launch', nr))
        return StartQuadServiceResponse(1)

    def get_wands(self):
        return self.get_nodes(is_quad=False)

    def get_quads(self):
        return self.get_nodes(is_quad=True)

    def get_nodes(self, is_quad):
        """
        helper for get matching nodes
        Using "/position" to remove duplicates
        """
        # Get all currently registered node names
        node_names = rospy.get_published_topics()
        id_objects = []
        for topic in node_names:
            if "/combo/" in topic[0]:
                if "/position" in topic[0]:
                    # match only one of the services to prevent duplicates
                    id_string = topic[0].split("/combo/")[1].split("/position")[0]

                    pattern1 = re.compile(r'^Sim_Wand_\d+$')
                    pattern2 = re.compile(r'^Wand\d+$')
                    pattern3 = re.compile(r'^(sim)?(\d+)$')
                    id_obj = IdObject()
                    id_obj.id = id_string

                    if pattern1.match(id_string):
                        id_obj.isSim = True
                        if not is_quad:
                            id_objects.append(id_obj)
                    elif pattern2.match(id_string):
                        id_obj.isSim = False
                        if not is_quad:
                            id_objects.append(id_obj)
                    elif pattern3.match(id_string):
                        groups = pattern3.search(id_string).groups()
                        if groups[0] is not None:
                            id_obj.isSim = True
                            if is_quad:
                                id_objects.append(id_obj)
                        else:
                            id_obj.isSim = False
                            if is_quad:
                                id_objects.append(id_obj)

        return id_objects

    def handle_fake(self, request) -> StartQuadServiceResponse:
        """
        handler for /combo/start_sim_quad service
        """
        nr = request.id
        self.launch_queue.append(('isse_crazy', 'fake_copter.launch', nr))
        return StartQuadServiceResponse(1)

    def handle_sim_wand(self, request) -> StartWandResponse:
        """
        handler for /combo/start_sim_wand service
        """
        nr = request.id
        self.launch_queue.append(('isse_crazy', 'sim_wand.launch', nr))
        return StartWandResponse(1)

    def handle_real_wand(self, request) -> StartWandResponse:
            """
            handler for /combo/start_rea_wand service
            """
            nr = request.id
            self.launch_queue.append(('isse_crazy', 'real_wand.launch', nr))
            return StartWandResponse(1)


if __name__ == "__main__":
    StartQuadServiceClass()
