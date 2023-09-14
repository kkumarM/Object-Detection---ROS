#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
bridge = CvBridge()

rospy.init_node("test_video_node")

publish_topic = rospy.get_param("/camera_input_topic", "/front_cam/camera/image")
video_path = rospy.get_param("/test_folder")
video_name = rospy.get_param("/test_video", "test.mp4")
video_rate = rospy.get_param("/test_video_rate", 10)

pub = rospy.Publisher(publish_topic, Image, queue_size=1)

r = rospy.Rate(video_rate)

while not rospy.is_shutdown():
    rospy.loginfo("Publishing Test Video!")
    cap = cv2.VideoCapture(video_path+video_name)
    while True:
        ret, frame = cap.read()
        if ret == True and not rospy.is_shutdown():
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(img_msg)
            r.sleep()
        else: 
            break
    if rospy.is_shutdown():
        break
    cap.release()
 
