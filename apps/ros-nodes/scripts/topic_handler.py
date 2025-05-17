#!/usr/bin/env python3

import rospy
import time
import threading
from tf2_msgs.msg import TFMessage
from rospy.exceptions import ROSTimeMovedBackwardsException

class TFConsolidator:
    """
    订阅 /tf 话题，合并所有收到的变换，定期清理超时变换，并发布到 /seed_ros_nodes/tf_consolidated。
    """

    def __init__(self):
        rospy.init_node('seed_ros_tf_consolidator')
        self.lock = threading.Lock()
        self.transforms = {}           # {child_frame_id: transform}
        self.transform_time = {}       # {child_frame_id: last_update_time}
        self.updated = False

        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.tf_pub = rospy.Publisher('/seed_ros_nodes/tf_consolidated', TFMessage, queue_size=1, latch=True)

        self.timeout = 10.0            # 超时时间（秒）
        self.publish_rate = 30         # 发布频率（Hz）
        self.clear_interval = 5        # 清理间隔（秒）

        rospy.loginfo("TF consolidator ready.")

    def tf_callback(self, msg):
        """收到TF消息时，更新缓存。"""
        with self.lock:
            now = time.time()
            for transform in msg.transforms:
                cid = transform.child_frame_id
                self.transforms[cid] = transform
                self.transform_time[cid] = now
            self.updated = True

    def clear_old_transforms(self):
        """清理超时未更新的TF。"""
        with self.lock:
            now = time.time()
            to_delete = [cid for cid, t in self.transform_time.items() if now - t > self.timeout]
            for cid in to_delete:
                parent = self.transforms[cid].header.frame_id
                del self.transforms[cid]
                del self.transform_time[cid]
                self.updated = True
                rospy.logwarn(f"Deleted old TF link: {parent} -> {cid}")

    def publish(self):
        """发布合并后的TF消息。"""
        with self.lock:
            if not self.updated:
                return
            msg = TFMessage()
            msg.transforms = list(self.transforms.values())
            self.updated = False
        self.tf_pub.publish(msg)

    def spin(self):
        """主循环：定期清理和发布。"""
        rate = rospy.Rate(self.publish_rate)
        last_clear = time.time()
        while not rospy.is_shutdown():
            try:
                now = time.time()
                if now - last_clear > self.clear_interval:
                    self.clear_old_transforms()
                    last_clear = now
                self.publish()
                rate.sleep()
            except ROSTimeMovedBackwardsException as e:
                rospy.logwarn(str(e))

if __name__ == "__main__":
    node = TFConsolidator()
    node.spin()
