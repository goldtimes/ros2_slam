
#!/usr/bin/env python3
# coding=utf-8
import rospy
import time
import os

from robot_manager.msg import slam_pose


class PubSlamPose:
    def __init__(self):
        self.pub = rospy.Publisher(
            "/init_pose", slam_pose, queue_size=1, latch=True
        )

        time.sleep(1)
        msg = slam_pose()
        msg.x = 1.87
        msg.y = 1.14
        msg.z = -0.42
        msg.roll = 0
        msg.pitch = 0
        msg.yaw = 1.6347589
        msg.level = 6
        msg.name = "MAP250923RCS645457827356942750"
        self.pub.publish(msg)
        rospy.loginfo("发布消息 {}".format(msg))



if __name__ == "__main__":
    rospy.init_node("pub_metainfo")
    node = PubSlamPose()
    time.sleep(1)
    exit(0)
