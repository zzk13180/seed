"""
Seed Core ROS 包 - Python 脚本模块

本模块提供机器人导航与控制的核心功能:
    - ServiceHandler: 系统服务处理器（节点管理、地图管理、文件操作）
    - TFConsolidator: TF 话题整合器（优化 WebSocket 传输）
    - PointManagerService: 导航点位管理器（SQLite 持久化）
"""

from .point_manager import PointManagerService as PointManagerService
from .service_handler import ServiceHandler as ServiceHandler
from .topic_handler import TFConsolidator as TFConsolidator

__all__ = ["ServiceHandler", "TFConsolidator", "PointManagerService"]
__version__ = "2.0.0"
