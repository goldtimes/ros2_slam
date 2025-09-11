'''
Author: lihang lihang@kilox.cn
Date: 2025-09-10 16:22:49
LastEditors: lihang lihang@kilox.cn
LastEditTime: 2025-09-11 09:48:01
FilePath: /fast_lvio_ws/src/open_slam/scripts/pub_map.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
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
        msg.x = 1
        msg.y = 1
        msg.z = 1
        msg.roll = 0
        msg.pitch = 0
        msg.yaw = 0
        msg.level = 6
        msg.name = "MAP20250530DDT358178911318183936"
        self.pub.publish(msg)
        rospy.loginfo("发布消息 {}".format(msg))



if __name__ == "__main__":
    rospy.init_node("pub_metainfo")
    node = PubSlamPose()
    time.sleep(1)
    exit(0)
