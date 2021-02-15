#!/usr/bin/env python

import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class DepthInterpolation:
    def __init__(self, node_name="depth_interpolation"):
        rospy.init_node(node_name, anonymous=True)

        self.sub_image_topic = rospy.get_param(
            "~sub_image_topic", "/camera/depth/image_rect_raw")
        self.pub_image_topic = rospy.get_param(
            "~pub_image_topic", "/depth_interpolated/image_raw")

        # self.sub_info_topic = rospy.get_param(
        #     "~sub_info_topic", "/camera/depth/camera_info")
        # self.pub_info_topic = rospy.get_param(
        #     "~pub_info_topic", "/depth_interpolated/camera_info")

        self.pub = rospy.Publisher(self.pub_image_topic, Image, queue_size=10)
        rospy.Subscriber(self.sub_image_topic, Image, self.callback)

        self.cv_bridge = CvBridge()

        rospy.spin()

    def callback(self, data):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(
                data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        sz = image.shape
        image_interpolated = image.copy()
        for i in range(sz[0]):
            for j in range(sz[1]):
                if image[i][j] == 0:
                    x1 = np.maximum(i-1, 0)
                    x2 = np.minimum(i+2, sz[0])
                    y1 = np.maximum(j-1, 0)
                    y2 = np.minimum(j+2, sz[1])
                    window = image[x1:x2, y1:y2]
                    image_interpolated[i][j] = np.amax(window)
                
        img_data = self.cv_bridge.cv2_to_imgmsg(image_interpolated)
        # img_data.header = data.header

        self.pub.publish(img_data)
        rospy.loginfo("done")


if __name__ == '__main__':
    try:
        di = DepthInterpolation()
    except rospy.ROSInterruptException:
        pass
