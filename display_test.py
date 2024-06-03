#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageDepthSubscriber:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribers for RGB and depth images
        self.rgb_sub = rospy.Subscriber(
            "/zedm/zed_node/rgb/image_rect_color", Image, self.rgb_callback
        )
        self.depth_sub = rospy.Subscriber(
            "/zedm/zed_node/depth/depth_registered", Image, self.depth_callback
        )
        self.rgb_image = None
        self.depth_image = None

    def rgb_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = depth_image
            # depth_image = np.array(depth_image, dtype=np.int16)
            print(np.max(depth_image))
            print(np.min(depth_image))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def display_images(self):
        while not rospy.is_shutdown():
            if self.rgb_image is not None:
                cv2.imshow("RGB Image", self.rgb_image)
            if self.depth_image is not None:
                cv2.imshow("Depth Image", self.depth_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("image_depth_subscriber", anonymous=True)
    ids = ImageDepthSubscriber()
    try:
        ids.display_images()
    except rospy.ROSInterruptException:
        pass
