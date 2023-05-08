#!/usr/bin/env python3
import json
import time
import csv
from typing import Tuple, List, Any

import roslaunch
import rospy
import re

import os

from isse_crazy.srv import StartQuadService, StartQuadServiceResponse
from isse_crazy.srv import GetQuads, GetQuadsResponse
from isse_crazy.srv import GetWands, GetWandsResponse
from isse_crazy.srv import StartWand, StartWandResponse
from isse_crazy.srv import NoArguments, NoArgumentsResponse
from isse_crazy.srv import StartTracking, StartTrackingResponse
from isse_crazy.srv import StopTracking, StopTrackingResponse
from isse_crazy.msg import IdObject
from pycrazyswarm.crazyflie import TimeHelper, CrazyflieServer, Crazyflie
from rospy import ROSException
from tf import TransformListener
from isse_crazy.msg import PoseVelStamped

from datetime import datetime


def get_crazyflies(transform_listener) -> Tuple[List[Any], str]:
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
            cf = Crazyflie(cf_id, initial_position, transform_listener)
            crazyflies.append(cf)
    except ROSException as e:
        error = str(e).replace("/set_group_mask", "")
    except Exception as e:
        error = str(e)
    return crazyflies, error


class ComboMainService:
    def __init__(self):
        rospy.init_node('combo_start_quad_service')
        # dict of {"<id>": dict{metadata}}
        self.tracking_properties = {}

        self.launch_queue = []
        self.launch_parents = []

        self.csv_header = ['seq', 'stamp', 'position', 'velocity']

        self.wand_tracking_active = False

        self.s1 = rospy.Service('start_real_quad', StartQuadService, self.handle_start_real_quad)
        self.s2 = rospy.Service('start_sim_quad', StartQuadService, self.handle_start_fake_quad)
        self.s3 = rospy.Service('get_quads', GetQuads, self.handle_get_quads)
        self.s4 = rospy.Service('start_sim_wand', StartWand, self.handle_start_sim_wand)
        self.s4 = rospy.Service('start_real_wand', StartWand, self.handle_start_real_wand)
        self.s5 = rospy.Service('get_wands', GetWands, self.handle_get_wands)
        # used to export data
        self.s7 = rospy.Service('start_tracking', StartTracking, self.handle_start_tracking)
        self.s8 = rospy.Service('stop_tracking', StartTracking, self.handle_stop_tracking)

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.pose_vel_stamped_subscribers = []

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

        # Unsubscribe from pose_vel_stamped pubs
        for sub in self.pose_vel_stamped_subscribers:
            sub.unregister()

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

    def on_new_quad_pose_vel_stamped(self, data, args):
        if self.__is_tracking_active_for(args["id"]):
            self.tracking_properties[args["id"]]["writer"].writerow(self.__pose_vel_to_csv_line(data))

    def on_new_wand_pose_vel_stamped(self, data, args):
        if self.wand_tracking_active:
            self.tracking_properties[args["id"]]["writer"].writerow(self.__pose_vel_to_csv_line(data))

    def __pose_vel_to_csv_line(self, data) -> [str]:
        return [
            str(data.header.seq),
            str(data.header.stamp.secs),
            self.__pose_to_string(data.pose),
            self.__velocity_to_string(data.velocity)
        ]

    def __pose_to_string(self, pose) -> str:
        x = f'{pose.position.x:.2f}'
        y = f'{pose.position.y:.2f}'
        z = f'{pose.position.z:.2f}'
        return x + ", " + y + ", " + z

    def __velocity_to_string(self, vel) -> str:
        x = f'{vel.linear.x:.4f}'
        y = f'{vel.linear.y:.4f}'
        z = f'{vel.linear.z:.4f}'
        return f"{x}, {y}, {z}"

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

    def handle_start_tracking(self, request) -> StartTrackingResponse:
        """
        handler for /combo/start_tracking
        """
        parsed_request = self.__update_tracking_properties(request=request, is_active=True)
        if not self.wand_tracking_active:
            self.__create_wand_files()

        self.wand_tracking_active = True
        file_name = self.__create_dirs_and_get_file_name(parsed_request)
        pose_file = open(file_name, 'w+', encoding='UTF8')
        csv_writer = csv.writer(pose_file, delimiter=";")
        self.tracking_properties[parsed_request["id"]]["file"] = pose_file
        self.tracking_properties[parsed_request["id"]]["writer"] = csv_writer
        csv_writer.writerow(self.csv_header)
        ret = StartTrackingResponse()
        ret.status = 1
        return ret

    def __create_wand_files(self):
        wands = self.get_wands()
        for wand in wands:
            meta = {"id": wand.id, "active": True}
            filename = self.__create_dirs_and_get_file_name(metadata=meta)
            wand_file = open(filename, 'w+', encoding='UTF8')
            csv_writer = csv.writer(wand_file, delimiter=";")
            self.tracking_properties[wand.id] = meta
            self.tracking_properties[wand.id]["file"] = wand_file
            self.tracking_properties[wand.id]["writer"] = csv_writer

    def __create_dirs_and_get_file_name(self, metadata) -> str:
        """
        take metadata lik {"id": "some_id"} and creates the correct file name
        """
        final_dir = self.__create_and_return_dirs()
        current_time_string = datetime.today().strftime("%H-%M-%S")
        return final_dir + "/" + metadata["id"] + "_" + current_time_string + ".csv"

    def __create_and_return_dirs(self) -> str:
        """
        creates the directories and returns the path
        """
        file_dir = os.path.dirname(os.path.abspath(__file__))
        current_date_string = datetime.today().strftime("%Y-%m-%d")
        final_dir = str(file_dir) + "/../" + '.tracking' + '/' + current_date_string
        if not os.path.exists(final_dir):
            os.makedirs(final_dir)
        return final_dir

    def __update_tracking_properties(self, request, is_active) -> {}:
        """
        updates the dict and returns the parsed_json
        """
        parsed_json = json.loads(request.metadata)
        self.tracking_properties[parsed_json['id']] = {
            "metadata": parsed_json,
            "active": is_active
        }
        return parsed_json

    def __is_tracking_active_for(self, identifier) -> bool:
        if identifier in self.tracking_properties:
            return self.tracking_properties[identifier]["active"]
        return False

    def __is_any_tracking_active(self) -> bool:
        return any(i["active"] for i in self.tracking_properties.values())

    def handle_stop_tracking(self, request) -> StartTrackingResponse:
        """
        handler for /combo/stop_tracking
        """
        parsed_request = self.__update_tracking_properties(request=request, is_active=False)
        # TODO(stefanb): Error on closing
        # self.tracking_properties[parsed_request["id"]]["file"].close()
        self.wand_tracking_active = self.__is_any_tracking_active()
        ret = StartTrackingResponse()
        ret.status = 1
        return ret

    def handle_start_real_quad(self, request) -> StartQuadServiceResponse:
        """
        handler for /combo/start_real_quad service
        """
        identifier = request.id
        self.launch_queue.append(('isse_crazy', 'real_copter.launch', identifier))
        self.subscribe_to_pose_vel_stamped(str(identifier))
        return StartQuadServiceResponse(1)

    def get_wands(self) -> [IdObject]:
        return self.get_nodes(is_quad=False)

    def get_quads(self) -> [IdObject]:
        return self.get_nodes(is_quad=True)

    def get_nodes(self, is_quad) -> [IdObject]:
        """
        helper for get matching nodes
        Using "/position" to remove duplicates
        """
        # Get all currently registered node names
        node_names = rospy.get_published_topics()
        id_objects = []
        for topic in node_names:
            name = topic[0]
            if "/combo/" in name:
                if "/position" in name:
                    # match only one of the services to prevent duplicates
                    id_string = name.split("/combo/")[1].split("/position")[0]

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

    def get_pose_vel_pubs(self) -> [str]:
        pubs = []
        node_names = rospy.get_published_topics()
        for topic in node_names:
            name = topic[0]
            if "/combo/" in name:
                if "/pose_vel_stamped" in name:
                    pubs.append(name)
        return pubs

    def get_pose_vel_pub(self, movable_id) -> str:
        pubs = self.get_pose_vel_pubs()
        return next((x for x in pubs if (movable_id in x)), None)

    def handle_start_fake_quad(self, request) -> StartQuadServiceResponse:
        """
        handler for /combo/start_sim_quad service
        """
        identifier = request.id
        self.launch_queue.append(('isse_crazy', 'fake_copter.launch', identifier))
        self.subscribe_to_pose_vel_stamped(str(identifier))
        return StartQuadServiceResponse(1)

    def handle_start_sim_wand(self, request) -> StartWandResponse:
        """
        handler for /combo/start_sim_wand service
        """
        identifier = request.id
        self.launch_queue.append(('isse_crazy', 'sim_wand.launch', identifier))
        self.subscribe_to_pose_vel_stamped(str(identifier))
        return StartWandResponse(1)

    def handle_start_real_wand(self, request) -> StartWandResponse:
        """
        handler for /combo/start_rea_wand service
        """
        identifier = request.id
        self.launch_queue.append(('isse_crazy', 'real_wand.launch', identifier))
        self.subscribe_to_pose_vel_stamped(str(identifier))
        return StartWandResponse(1)

    def subscribe_to_pose_vel_stamped(self, movable_id):
        time.sleep(2)
        pub = self.get_pose_vel_pub(movable_id)
        if pub is None:
            rospy.logerr("Pub not found with id " + movable_id)
            return
        subscriber = rospy.Subscriber(pub, PoseVelStamped, self.on_new_quad_pose_vel_stamped, {"id": movable_id})
        self.pose_vel_stamped_subscribers.append(subscriber)


if __name__ == "__main__":
    ComboMainService()
