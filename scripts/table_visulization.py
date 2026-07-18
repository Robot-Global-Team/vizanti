#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from visualization_msgs.msg import Marker, MarkerArray

def create_marker(id, name, x, y):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "markers"
    marker.id = id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.text = name
    return marker

def callback(data):
    global floor_data
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    floor_data = json.loads(data.data)

def timer_callback(event):
    if floor_data:
        marker_array = MarkerArray()
        for item in floor_data:
            marker = create_marker(item['index'], item['name'], item['x'], item['y'])
            marker_array.markers.append(marker)
        marker_pub.publish(marker_array)

def listener():
    global marker_pub, floor_data
    floor_data = None
    rospy.init_node('floor_listener', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    rospy.Subscriber('/sirbot1/floor_topic', String, callback)
    rospy.Timer(rospy.Duration(1), timer_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
