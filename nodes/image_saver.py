#!/usr/bin/python
"""
This ROS node caputure the image in PFC and save as a file in the directory ~/tmp_imgs/.
when the image file is successfully saved, this ROS node publish the file path to the topic /image_updated
"""

import rospy
from std_msgs.msg import  String
import time, os, subprocess, sys
import cv2

DEVICE_VIDEO_PATH = '/dev/video'


def save_image(device_name, TMP_IMG_PATH):
    is_updated = False
    curr_time = int(time.time())
    filename = '{}.png'.format(str(curr_time))
    try:
        device_number = int(device_name.strip(DEVICE_VIDEO_PATH))
    except:
        rospy.logwarn("incorrect device name")
    video_cap = cv2.VideoCapture(device_number)
    if not video_cap.isOpened():
        rospy.logwarn('{} cannot be open'.format(device_name))
    else:
        ret, img = video_cap.read()
        if ret:
            cv2.imwrite(TMP_IMG_PATH  + filename, img)
    video_cap.release()
    cv2.destroyAllWindows()

    if os.path.exists(TMP_IMG_PATH + filename):
        is_updated = True
        for img_file in os.listdir(TMP_IMG_PATH):
            if img_file != filename:
                os.remove(TMP_IMG_PATH + img_file)

    return is_updated, TMP_IMG_PATH + filename


if __name__ == '__main__':
    rospy.init_node('image_saver')
    pub = rospy.Publisher('aerial_image/image_updated', String, queue_size=10)
    delay_seconds = rospy.get_param("~min_update_interval",60)
    device_name = rospy.get_param("~device_name",'/dev/video0')
    TMP_IMG_PATH = '/home/pi/tmp_imgs/{}/'.format(rospy.get_param("~camera_name",'aerial_image'))

    while True:
        is_updated, file_path = save_image(device_name,TMP_IMG_PATH)
        if is_updated:
            rospy.loginfo(file_path)
            pub.publish(file_path)
        time.sleep(delay_seconds)   # Delay for 1 minute (60 seconds).




