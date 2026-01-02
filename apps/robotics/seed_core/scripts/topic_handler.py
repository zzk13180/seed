#!/usr/bin/env python3
"""Seed Core TF 话题处理器 - 将分散的 TF 消息整合为统一话题发布"""

from dataclasses import dataclass, field
import threading
import time
from typing import Dict

from geometry_msgs.msg import TransformStamped
import rospy
from rospy.exceptions import ROSTimeMovedBackwardsException
from tf2_msgs.msg import TFMessage


class TFConfig:
    """TF 处理器配置常量"""

    DEFAULT_PUBLISH_RATE = 30
    DEFAULT_TIMEOUT = 10.0
    DEFAULT_CLEAR_INTERVAL = 5.0
    INPUT_TOPIC = "/tf"
    OUTPUT_TOPIC = "/seed_core/tf_consolidated"


@dataclass
class TransformEntry:
    """TF 变换条目，含变换数据和更新时间戳"""

    transform: TransformStamped
    last_update: float = field(default_factory=time.time)

    def is_expired(self, timeout: float) -> bool:
        """检查该变换是否已超时"""
        return time.time() - self.last_update > timeout


class TransformCache:
    """线程安全的 TF 变换缓存管理类"""

    def __init__(self, timeout: float = TFConfig.DEFAULT_TIMEOUT):
        """初始化变换缓存"""
        self._transforms: Dict[str, TransformEntry] = {}
        self._lock = threading.Lock()
        self._timeout = timeout
        self._updated = False

    @property
    def has_updates(self) -> bool:
        """检查是否有待发布的更新"""
        with self._lock:
            return self._updated

    def update(self, transforms: list) -> None:
        """更新缓存中的变换数据"""
        with self._lock:
            current_time = time.time()
            for transform in transforms:
                child_frame_id = transform.child_frame_id
                self._transforms[child_frame_id] = TransformEntry(
                    transform=transform, last_update=current_time
                )
            self._updated = True

    def get_all_transforms(self) -> list:
        """获取所有缓存的变换并清除更新标志"""
        with self._lock:
            self._updated = False
            return [entry.transform for entry in self._transforms.values()]

    def clear_expired(self) -> int:
        """清理超时的变换，返回被清理的数量"""
        with self._lock:
            expired_keys = []
            for child_frame_id, entry in self._transforms.items():
                if entry.is_expired(self._timeout):
                    expired_keys.append(child_frame_id)

            for key in expired_keys:
                parent_frame = self._transforms[key].transform.header.frame_id
                del self._transforms[key]
                rospy.logwarn(f"已清理超时的 TF 链接: {parent_frame} -> {key}")

            if expired_keys:
                self._updated = True

            return len(expired_keys)

    @property
    def count(self) -> int:
        """获取当前缓存的变换数量"""
        with self._lock:
            return len(self._transforms)


class TFConsolidator:
    """订阅 /tf 话题，整合变换数据，以固定频率发布整合后的 TF 消息"""

    def __init__(self):
        """初始化 TF 整合器"""
        rospy.init_node("seed_core_tf_consolidator")
        self._publish_rate = rospy.get_param("~publish_rate", TFConfig.DEFAULT_PUBLISH_RATE)
        self._timeout = rospy.get_param("~tf_timeout", TFConfig.DEFAULT_TIMEOUT)
        self._clear_interval = rospy.get_param("~clear_interval", TFConfig.DEFAULT_CLEAR_INTERVAL)
        self._cache = TransformCache(timeout=self._timeout)
        self._last_clear_time = time.time()

        self._tf_subscriber = rospy.Subscriber(
            TFConfig.INPUT_TOPIC,
            TFMessage,
            self._on_tf_received,
            queue_size=100,
        )
        self._tf_publisher = rospy.Publisher(
            TFConfig.OUTPUT_TOPIC, TFMessage, queue_size=1, latch=True
        )

        rospy.loginfo("=== Seed Core TF 整合器启动 ===")
        rospy.loginfo(f"发布频率: {self._publish_rate} Hz")
        rospy.loginfo(f"超时时间: {self._timeout} 秒")
        rospy.loginfo(f"清理间隔: {self._clear_interval} 秒")

    def _on_tf_received(self, msg: TFMessage) -> None:
        """处理接收到的 TF 消息"""
        if msg.transforms:
            self._cache.update(msg.transforms)

    def _publish_transforms(self) -> None:
        """发布整合后的 TF 消息"""
        if not self._cache.has_updates:
            return

        transforms = self._cache.get_all_transforms()

        if transforms:
            msg = TFMessage()
            msg.transforms = transforms
            self._tf_publisher.publish(msg)

    def _check_and_clear_expired(self) -> None:
        """根据配置的清理间隔定期清理超时变换"""
        current_time = time.time()

        if current_time - self._last_clear_time > self._clear_interval:
            cleared_count = self._cache.clear_expired()
            self._last_clear_time = current_time

            if cleared_count > 0:
                rospy.loginfo(f"已清理 {cleared_count} 个超时的 TF 变换")

    def spin(self) -> None:
        """主循环，定期清理超时变换并发布整合消息"""
        rate = rospy.Rate(self._publish_rate)
        rospy.loginfo("TF 整合器开始运行")

        while not rospy.is_shutdown():
            try:
                self._check_and_clear_expired()
                self._publish_transforms()
                rate.sleep()
            except ROSTimeMovedBackwardsException as e:
                rospy.logwarn(f"ROS 时间回退: {e}")
            except rospy.exceptions.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"主循环异常: {e}")

        rospy.loginfo("TF 整合器已停止")

    def get_stats(self) -> Dict:
        """获取当前统计信息"""
        return {
            "cached_transforms": self._cache.count,
            "publish_rate": self._publish_rate,
            "timeout": self._timeout,
        }


if __name__ == "__main__":
    try:
        consolidator = TFConsolidator()
        consolidator.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF 整合器已关闭")
    except Exception as e:
        rospy.logerr(f"TF 整合器异常: {e}")
        raise
