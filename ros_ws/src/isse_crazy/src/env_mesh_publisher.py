#!/usr/bin/env python3
import rospy
import visualization_msgs.msg

if __name__ == "__main__":
    rospy.init_node('env_mesh_publisher')
    rate = rospy.Rate(1)
    marker_pub = rospy.Publisher("/visualization_marker", visualization_msgs.msg.Marker, queue_size=1)

    while not rospy.is_shutdown():
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = "world"
        marker.ns = "Flight arena"
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = "package://isse_crazy/meshes/arena.dae"
        marker.mesh_use_embedded_materials = True
        marker.id = 0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.pose.orientation.w = 1.0
        marker_pub.publish(marker)
        rate.sleep()