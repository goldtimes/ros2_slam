'''
Author: lihang lihang@kilox.cn
Date: 2025-09-10 16:22:49
LastEditors: lihang lihang@kilox.cn
LastEditTime: 2025-09-10 16:31:06
FilePath: /fast_lvio_ws/src/open_slam/scripts/pub_map.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3
# coding=utf-8
import rospy
import time
import os

from robot_manager.msg import metaset_info


class PubMetaInfo:
    def __init__(self):
        self.pub = rospy.Publisher(
            "metaset_info", metaset_info, queue_size=1, latch=True
        )

        time.sleep(1)
        map_dir = rospy.get_param(
            "/slam_node/loc_map_dir", "/home/kilox/maps/6office"
        )
        map_name = rospy.get_param(
            "/slam_node/loc_map_name", "MAP20250530DDT358178911318183936"
        )

        msg = metaset_info()
        msg.labels = ["ABC"]
        msg.v_name.append(map_name)
        # print(map_dir)
        # if os.path.exists(map_dir):
        #     for file in os.listdir(map_dir):
        #         if file.startswith("MAP"):
        #             msg.v_name.append(file)

        self.pub.publish(msg)
        rospy.loginfo("发布消息 {}".format(msg))

        # 5秒后发布初始化定位信息


if __name__ == "__main__":
    rospy.init_node("pub_metainfo")
    node = PubMetaInfo()
    time.sleep(5)
    exit(0)
