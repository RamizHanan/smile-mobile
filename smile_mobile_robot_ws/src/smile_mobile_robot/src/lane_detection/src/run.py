from ctypes import *
import sys
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from lanedetector import LaneDetector

bridge = CvBridge()
detector = LaneDetector()
pub = rospy.Publisher('LaneDetection', String, queue_size=10)
laneinfo = 'None'


def callback(image_msg):
    global laneinfo
    try:
        img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img, laneinfo = detector.detect_img(img)
        rospy.loginfo(laneinfo)
        pub.publish(laneinfo)
        cv2.imshow("img", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            sys.exit
    except CvBridgeError as e:
        print(e)


if __name__ == "__main__":
    rospy.init_node('cameraYOLO')
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback, queue_size=1)
    pub.publish(laneinfo)
    rospy.spin()
