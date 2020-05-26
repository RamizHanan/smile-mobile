# Simple talker demo that published std_msgs/Strings messages
# to the 'LaneDetection' topic

import rospy
from std_msgs.msg import String
from lanedetector import LaneDetector


def talker():
    detector = LaneDetector()
    pub = rospy.Publisher('LaneDetection', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        lane_info = "%s" % detector.detect_img('testCurveLane2.jpg')
        rospy.loginfo(lane_info)
        pub.publish(lane_info)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
