#!/usr/bin/env python3
"""Seed Core 点位管理器 - 使用 SQLite 持久化存储导航点位数据"""

import base64
from contextlib import contextmanager
from dataclasses import dataclass, field
from enum import Enum
import json
import os
import sqlite3
from threading import Lock
from typing import Any, Dict, List, Optional, Tuple

import rospy

from seed_core.srv import (
    AddPoint,
    AddPointResponse,
    DeletePoint,
    DeletePointResponse,
    GetPoint,
    GetPointResponse,
    ListPoints,
    ListPointsResponse,
)


class DatabaseConfig:
    """数据库配置常量"""

    DEFAULT_DB_DIR = "~/.seed/data"
    DB_FILENAME = "point_markers.db"
    TABLE_NAME = "point_markers"


class PointType(Enum):
    """点位类型枚举"""

    NAVIGATION = "navigation"

    @classmethod
    def from_string(cls, value: str) -> Optional["PointType"]:
        """从字符串转换为枚举值"""
        try:
            return cls(value.lower())
        except ValueError:
            return None

    @classmethod
    def is_valid(cls, value: str) -> bool:
        """检查类型字符串是否有效"""
        return cls.from_string(value) is not None


@dataclass
class PointMarker:
    """点位数据类，包含位置、朝向、地图关联等信息"""

    # 基本信息
    id: Optional[int]
    name: str
    point_type: PointType

    # 位置信息
    x: float
    y: float
    z: float

    # 朝向信息（四元数）
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float

    # 关联信息
    map_name: str

    # 扩展信息
    description: str = ""
    metadata: Dict = field(default_factory=dict)
    tags: List[str] = field(default_factory=list)
    enabled: bool = True

    def to_dict(self) -> Dict[str, Any]:
        """将点位对象转换为字典，用于 JSON 序列化"""
        return {
            "id": self.id,
            "name": self.name,
            "point_type": self.point_type.value,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "orientation_x": self.orientation_x,
            "orientation_y": self.orientation_y,
            "orientation_z": self.orientation_z,
            "orientation_w": self.orientation_w,
            "map_name": self.map_name,
            "description": self.description,
            "metadata": self.metadata,
            "tags": self.tags,
            "enabled": self.enabled,
        }

    @classmethod
    def from_db_row(cls, row: tuple) -> "PointMarker":
        """从数据库行元组创建 PointMarker 实例"""
        # 解析 JSON 字段
        try:
            metadata = json.loads(row[12]) if row[12] else {}
        except json.JSONDecodeError:
            metadata = {}

        try:
            tags = json.loads(row[13]) if row[13] else []
        except json.JSONDecodeError:
            tags = []

        return cls(
            id=row[0],
            name=row[1],
            point_type=PointType(row[2]),
            x=row[3],
            y=row[4],
            z=row[5],
            orientation_x=row[6],
            orientation_y=row[7],
            orientation_z=row[8],
            orientation_w=row[9],
            map_name=row[10],
            description=row[11] or "",
            metadata=metadata,
            tags=tags,
            enabled=bool(row[14]),
        )


def safe_base64_decode(encoded: str) -> str:
    """安全的 Base64 解码，失败时返回原字符串"""
    if not encoded:
        return ""

    try:
        return base64.b64decode(encoded).decode("utf-8")
    except Exception:
        return encoded


def safe_base64_encode(text: str) -> str:
    """安全的 Base64 编码，失败时返回原字符串"""
    if not text:
        return ""

    try:
        return base64.b64encode(text.encode("utf-8")).decode("ascii")
    except Exception:
        return text


class PointDatabase:
    """点位数据库管理类，线程安全，负责 SQLite 的 CRUD 操作"""

    def __init__(self, db_path: str):
        """初始化数据库管理器"""
        self._db_path = os.path.expanduser(db_path)
        self._lock = Lock()

        # 确保目录存在
        db_dir = os.path.dirname(self._db_path)
        if db_dir and not os.path.exists(db_dir):
            os.makedirs(db_dir, exist_ok=True)

        # 初始化数据库
        self._init_database()

        rospy.loginfo(f"[PointDatabase] 数据库初始化完成: {self._db_path}")

    @contextmanager
    def _get_connection(self):
        """获取数据库连接的上下文管理器"""
        conn = sqlite3.connect(self._db_path)
        try:
            yield conn
        finally:
            conn.close()

    def _init_database(self) -> None:
        """初始化数据库，创建表结构和索引"""
        with self._lock, self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                    CREATE TABLE IF NOT EXISTS point_markers (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        name TEXT NOT NULL,
                        point_type TEXT NOT NULL,
                        x REAL NOT NULL,
                        y REAL NOT NULL,
                        z REAL NOT NULL,
                        orientation_x REAL NOT NULL,
                        orientation_y REAL NOT NULL,
                        orientation_z REAL NOT NULL,
                        orientation_w REAL NOT NULL,
                        map_name TEXT NOT NULL,
                        description TEXT DEFAULT '',
                        metadata TEXT DEFAULT '{}',
                        tags TEXT DEFAULT '[]',
                        enabled BOOLEAN DEFAULT TRUE,
                        UNIQUE(name, point_type, map_name)
                    )
                """)
            cursor.execute("""
                    CREATE INDEX IF NOT EXISTS idx_point_type
                    ON point_markers(point_type)
                """)
            cursor.execute("""
                    CREATE INDEX IF NOT EXISTS idx_map_name
                    ON point_markers(map_name)
                """)
            cursor.execute("""
                    CREATE INDEX IF NOT EXISTS idx_enabled
                    ON point_markers(enabled)
                """)
            cursor.execute("""
                    CREATE INDEX IF NOT EXISTS idx_name_type
                    ON point_markers(name, point_type)
                """)
            conn.commit()

    def add_point(self, point: PointMarker) -> Tuple[bool, str, Optional[int]]:
        """添加新点位，返回 (是否成功, 消息, 点位ID)"""
        with self._lock:
            try:
                with self._get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(
                        """
                        SELECT id FROM point_markers
                        WHERE name = ? AND point_type = ? AND map_name = ?
                    """,
                        (point.name, point.point_type.value, point.map_name),
                    )

                    if cursor.fetchone():
                        return (
                            False,
                            f"点位 '{point.name}' 在地图 '{point.map_name}' 中已存在",
                            None,
                        )

                    cursor.execute(
                        """
                        INSERT INTO point_markers (
                            name, point_type, x, y, z,
                            orientation_x, orientation_y, orientation_z, orientation_w,
                            map_name, description, metadata, tags, enabled
                        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                        (
                            point.name,
                            point.point_type.value,
                            point.x,
                            point.y,
                            point.z,
                            point.orientation_x,
                            point.orientation_y,
                            point.orientation_z,
                            point.orientation_w,
                            point.map_name,
                            point.description,
                            json.dumps(point.metadata, ensure_ascii=False),
                            json.dumps(point.tags, ensure_ascii=False),
                            point.enabled,
                        ),
                    )

                    point_id = cursor.lastrowid
                    conn.commit()

                    rospy.loginfo(f"[PointDatabase] 添加点位成功: {point.name} (ID: {point_id})")
                    return (True, f"成功添加点位: {point.name}", point_id)

            except sqlite3.IntegrityError as e:
                rospy.logerr(f"[PointDatabase] 约束冲突: {e}")
                return (False, "添加点位失败：约束冲突", None)

            except Exception as e:
                rospy.logerr(f"[PointDatabase] 添加点位失败: {e}")
                return (False, f"添加点位失败: {str(e)}", None)

    def delete_point(
        self,
        point_id: Optional[int] = None,
        name: Optional[str] = None,
        point_type: Optional[PointType] = None,
    ) -> Tuple[bool, str]:
        """删除点位，支持通过 ID 或名称+类型删除"""
        with self._lock:
            try:
                with self._get_connection() as conn:
                    cursor = conn.cursor()

                    if point_id and point_id > 0:
                        cursor.execute("DELETE FROM point_markers WHERE id = ?", (point_id,))
                        identifier = f"ID {point_id}"
                    elif name and point_type:
                        cursor.execute(
                            "DELETE FROM point_markers WHERE name = ? AND point_type = ?",
                            (name, point_type.value),
                        )
                        identifier = f"'{name}' ({point_type.value})"
                    else:
                        return (False, "必须提供点位 ID 或名称和类型")

                    if cursor.rowcount == 0:
                        return (False, f"点位 {identifier} 不存在")

                    conn.commit()
                    rospy.loginfo(f"[PointDatabase] 删除点位成功: {identifier}")
                    return (True, f"成功删除点位: {identifier}")

            except Exception as e:
                rospy.logerr(f"[PointDatabase] 删除点位失败: {e}")
                return (False, f"删除点位失败: {str(e)}")

    def get_point(
        self,
        point_id: Optional[int] = None,
        name: Optional[str] = None,
        point_type: Optional[PointType] = None,
    ) -> Optional[PointMarker]:
        """获取单个点位，支持通过 ID 或名称+类型查询"""
        with self._lock:
            try:
                with self._get_connection() as conn:
                    cursor = conn.cursor()

                    if point_id and point_id > 0:
                        cursor.execute("SELECT * FROM point_markers WHERE id = ?", (point_id,))
                    elif name and point_type:
                        cursor.execute(
                            "SELECT * FROM point_markers WHERE name = ? AND point_type = ?",
                            (name, point_type.value),
                        )
                    else:
                        rospy.logwarn("[PointDatabase] 查询点位需要 ID 或名称+类型")
                        return None

                    row = cursor.fetchone()
                    if row:
                        return PointMarker.from_db_row(row)
                    return None

            except Exception as e:
                rospy.logerr(f"[PointDatabase] 查询点位失败: {e}")
                return None

    def get_points(
        self,
        point_type: Optional[PointType] = None,
        map_name: Optional[str] = None,
        enabled_only: bool = True,
    ) -> List[PointMarker]:
        """获取点位列表，支持按类型、地图、启用状态过滤"""
        with self._lock:
            try:
                with self._get_connection() as conn:
                    cursor = conn.cursor()
                    conditions = []
                    params = []

                    if point_type:
                        conditions.append("point_type = ?")
                        params.append(point_type.value)

                    if map_name:
                        conditions.append("map_name = ?")
                        params.append(map_name)

                    if enabled_only:
                        conditions.append("enabled = 1")

                    query = "SELECT * FROM point_markers"
                    if conditions:
                        query += " WHERE " + " AND ".join(conditions)
                    query += " ORDER BY point_type, name ASC"

                    cursor.execute(query, params)
                    rows = cursor.fetchall()

                    return [PointMarker.from_db_row(row) for row in rows]

            except Exception as e:
                rospy.logerr(f"[PointDatabase] 查询点位列表失败: {e}")
                return []


class PointManagerService:
    """点位管理 ROS 服务类，提供点位的增删改查接口"""

    def __init__(self):
        """初始化点位管理服务"""
        rospy.init_node("seed_core_point_manager")
        rospy.loginfo("=== Seed Core 点位管理服务启动 ===")

        db_dir = rospy.get_param("~database_dir", DatabaseConfig.DEFAULT_DB_DIR)
        db_path = os.path.join(os.path.expanduser(db_dir), DatabaseConfig.DB_FILENAME)
        self._database = PointDatabase(db_path)
        self._register_services()
        rospy.loginfo("=== 点位管理服务就绪 ===")

    def _register_services(self) -> None:
        """注册所有 ROS 服务"""
        rospy.Service("seed_core/point_manager/add_point", AddPoint, self._handle_add_point)
        rospy.Service(
            "seed_core/point_manager/delete_point", DeletePoint, self._handle_delete_point
        )
        rospy.Service("seed_core/point_manager/get_point", GetPoint, self._handle_get_point)
        rospy.Service("seed_core/point_manager/list_points", ListPoints, self._handle_list_points)

    def _handle_add_point(self, req) -> AddPointResponse:
        """处理添加点位服务请求"""
        response = AddPointResponse()

        try:
            if not PointType.is_valid(req.point_type):
                response.success = False
                response.message = safe_base64_encode(f"不支持的点位类型: {req.point_type}")
                return response

            point_type = PointType.from_string(req.point_type)
            name = safe_base64_decode(req.name)
            map_name = safe_base64_decode(req.map_name)
            description = safe_base64_decode(req.description)
            metadata = {}
            tags = []

            if req.metadata:
                try:
                    metadata = json.loads(req.metadata)
                except json.JSONDecodeError:
                    rospy.logwarn(f"[PointManager] 无法解析 metadata: {req.metadata}")

            if req.tags:
                try:
                    tags = json.loads(req.tags)
                except json.JSONDecodeError:
                    rospy.logwarn(f"[PointManager] 无法解析 tags: {req.tags}")

            point = PointMarker(
                id=None,
                name=name,
                point_type=point_type,
                x=req.x,
                y=req.y,
                z=req.z,
                orientation_x=req.orientation_x,
                orientation_y=req.orientation_y,
                orientation_z=req.orientation_z,
                orientation_w=req.orientation_w,
                map_name=map_name,
                description=description,
                metadata=metadata,
                tags=tags,
                enabled=req.enabled,
            )

            success, message, point_id = self._database.add_point(point)

            response.success = success
            response.message = safe_base64_encode(message)
            response.point_id = point_id if point_id else 0

        except Exception as e:
            response.success = False
            response.message = safe_base64_encode(f"添加点位失败: {str(e)}")
            rospy.logerr(f"[PointManager] 添加点位异常: {e}")

        return response

    def _handle_delete_point(self, req) -> DeletePointResponse:
        """处理删除点位服务请求"""
        response = DeletePointResponse()

        try:
            point_type = None
            if req.point_type:
                point_type = PointType.from_string(req.point_type)

            point_id = req.point_id if req.point_id > 0 else None

            success, message = self._database.delete_point(
                point_id=point_id, name=req.name if req.name else None, point_type=point_type
            )

            response.success = success
            response.message = message

        except Exception as e:
            response.success = False
            response.message = f"删除点位失败: {str(e)}"
            rospy.logerr(f"[PointManager] 删除点位异常: {e}")

        return response

    def _handle_get_point(self, req) -> GetPointResponse:
        """处理获取点位服务请求"""
        response = GetPointResponse()

        try:
            point_type = None
            if req.point_type:
                point_type = PointType.from_string(req.point_type)

            point_id = req.point_id if req.point_id > 0 else None
            name = safe_base64_decode(req.name) if req.name else None
            point = self._database.get_point(point_id=point_id, name=name, point_type=point_type)

            if point:
                response.success = True
                response.message = "查询成功"
                response.point_id = point.id or 0
                response.name = safe_base64_encode(point.name)
                response.point_type = point.point_type.value
                response.x = point.x
                response.y = point.y
                response.z = point.z
                response.orientation_x = point.orientation_x
                response.orientation_y = point.orientation_y
                response.orientation_z = point.orientation_z
                response.orientation_w = point.orientation_w
                response.map_name = safe_base64_encode(point.map_name)
                response.description = safe_base64_encode(point.description)
                response.metadata = json.dumps(point.metadata, ensure_ascii=True)
                response.tags = json.dumps(point.tags, ensure_ascii=True)
                response.enabled = point.enabled
            else:
                response.success = False
                response.message = "点位不存在"

        except Exception as e:
            response.success = False
            response.message = f"查询点位失败: {str(e)}"
            rospy.logerr(f"[PointManager] 查询点位异常: {e}")

        return response

    def _handle_list_points(self, req) -> ListPointsResponse:
        """处理列出点位服务请求"""
        response = ListPointsResponse()

        try:
            point_type = None
            if req.point_type:
                point_type = PointType.from_string(req.point_type)

            map_name = safe_base64_decode(req.map_name) if req.map_name else None
            points = self._database.get_points(
                point_type=point_type, map_name=map_name, enabled_only=req.enabled_only
            )

            response.success = True
            response.message = f"找到 {len(points)} 个点位"
            response.count = len(points)

            points_data = []
            for point in points:
                point_dict = point.to_dict()
                point_dict["name"] = safe_base64_encode(point_dict["name"])
                point_dict["map_name"] = safe_base64_encode(point_dict["map_name"])
                point_dict["description"] = safe_base64_encode(point_dict["description"])
                points_data.append(point_dict)

            response.points_json = json.dumps(points_data, ensure_ascii=True, indent=2)

        except Exception as e:
            response.success = False
            response.message = f"查询点位列表失败: {str(e)}"
            response.count = 0
            response.points_json = "[]"
            rospy.logerr(f"[PointManager] 查询点位列表异常: {e}")

        return response


if __name__ == "__main__":
    try:
        service = PointManagerService()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("点位管理服务已关闭")
    except Exception as e:
        rospy.logerr(f"点位管理服务异常: {e}")
        raise
