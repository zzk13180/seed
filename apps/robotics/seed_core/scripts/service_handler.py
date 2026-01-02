#!/usr/bin/env python3
"""Seed Core 服务处理器 - 提供节点管理、地图管理、文件操作等核心服务"""

import contextlib
import copy
from dataclasses import dataclass
from enum import Enum
import fcntl
import json
import os
import shlex
import signal
import subprocess
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from PIL import Image
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from seed_core.srv import (
    ListExecutables,
    ListExecutablesResponse,
    ListMaps,
    ListMapsResponse,
    ListPackages,
    ListPackagesResponse,
    LoadMap,
    LoadMapResponse,
    ManageNode,
    ManageNodeResponse,
    ReadPgm,
    ReadPgmResponse,
    ReadTextFile,
    ReadTextFileResponse,
    SaveMap,
    SaveMapResponse,
    UpdateJson,
    UpdateJsonResponse,
    UpdatePgm,
    UpdatePgmResponse,
)


class NodeName:
    """节点名称常量"""

    MOVE_BASE = "/move_base"
    SLAM_TOOLBOX = "/slam_toolbox"


class LaunchFile:
    """Launch 文件名称常量"""

    NAVIGATION = "navigation.launch"
    MAPPING = "mapping.launch"


class ProcessState(Enum):
    """进程状态枚举"""

    RUNNING = "running"
    STOPPED = "stopped"
    UNKNOWN = "unknown"


@dataclass
class ProcessInfo:
    """进程信息"""

    process: subprocess.Popen
    command: List[str]
    key: str


class BaseManager:
    """管理器基类，提供统一的日志记录方法"""

    def _log_info(self, message: str) -> None:
        """记录信息日志"""
        rospy.loginfo(f"[{self.__class__.__name__}] {message}")

    def _log_warn(self, message: str) -> None:
        """记录警告日志"""
        rospy.logwarn(f"[{self.__class__.__name__}] {message}")

    def _log_error(self, message: str) -> None:
        """记录错误日志"""
        rospy.logerr(f"[{self.__class__.__name__}] {message}")


class ProcessManager(BaseManager):
    """进程管理器，管理 ROS launch 文件和节点的生命周期"""

    def __init__(self):
        """初始化进程管理器"""
        self._processes: Dict[str, ProcessInfo] = {}
        self._log_info("进程管理器初始化完成")
    @property
    def active_processes(self) -> Dict[str, ProcessInfo]:
        """获取所有活动进程"""
        return self._processes.copy()

    def start_launch(self, key: str, command: List[str]) -> Tuple[bool, str]:
        """启动 launch 文件或 ROS 命令"""
        self._log_info(f"尝试启动: key={key}, cmd={' '.join(command)}")

        if key in self._processes:
            proc_info = self._processes[key]
            if proc_info.process.poll() is None:
                self._log_warn(f"{key} 已在运行中 (pid {proc_info.process.pid})")
                return False, f"{key} 已在运行中 (pid {proc_info.process.pid})"
            else:
                self._log_info(f"{key} 之前的进程已退出，移除记录")
                del self._processes[key]

        try:
            env = copy.deepcopy(os.environ)

            with open(os.devnull, "w") as devnull:
                proc = subprocess.Popen(
                    command,
                    stdout=devnull,
                    stderr=devnull,
                    preexec_fn=os.setpgrp,  # 创建新进程组，便于后续整体终止
                    env=env,
                )

            self._processes[key] = ProcessInfo(process=proc, command=command, key=key)

            self._log_info(f"启动 {key} 成功 (pid {proc.pid})")
            return True, f"启动 {key} 成功 (pid {proc.pid})"

        except Exception as e:
            self._log_error(f"启动 {key} 失败: {e}")
            return False, f"启动 {key} 失败: {str(e)}"

    def kill_process(self, key: str, timeout: float = 3.0) -> Tuple[bool, str]:
        """终止指定进程，先发 SIGTERM，超时后发 SIGKILL"""
        self._log_info(f"请求终止进程: {key}")

        if key not in self._processes:
            self._log_warn(f"进程 {key} 不存在")
            return False, f"进程 {key} 不存在"

        proc_info = self._processes[key]
        proc = proc_info.process

        if proc.poll() is not None:
            self._log_info(f"进程 {key} 已经退出")
            del self._processes[key]
            return True, f"进程 {key} 已经退出"

        try:
            pgid = os.getpgid(proc.pid)

            if pgid <= 1:  # 安全检查：不要杀死系统进程组
                self._log_warn(f"跳过系统进程组: {pgid}")
                del self._processes[key]
                return False, f"无法终止系统进程组: {pgid}"

            self._log_info(f"发送 SIGTERM 到进程组 {pgid}")
            os.killpg(pgid, signal.SIGTERM)

            for _ in range(int(timeout * 10)):
                if proc.poll() is not None:
                    break
                rospy.sleep(0.1)

            if proc.poll() is None:
                self._log_warn(f"进程组 {key} 未响应 SIGTERM，发送 SIGKILL")
                os.killpg(pgid, signal.SIGKILL)
                for _ in range(20):
                    if proc.poll() is not None:
                        break
                    rospy.sleep(0.1)

            with contextlib.suppress(subprocess.TimeoutExpired):
                proc.wait(timeout=1)

            del self._processes[key]
            self._log_info(f"进程 {key} 已成功终止 (pid {proc.pid})")
            return True, f"进程 {key} 已成功终止"

        except Exception as e:
            self._log_error(f"终止进程 {key} 失败: {e}")
            if key in self._processes:
                del self._processes[key]
            return False, f"终止进程 {key} 失败: {str(e)}"

    def kill_ros_node(self, node_name: str) -> Tuple[bool, str]:
        """使用 rosnode kill 命令终止 ROS 节点"""
        self._log_info(f"请求终止 ROS 节点: {node_name}")

        try:
            result = subprocess.run(["rosnode", "list"], capture_output=True, text=True, timeout=5)

            nodes = result.stdout.strip().split("\n")
            if node_name not in nodes:
                self._log_info(f"节点 {node_name} 不存在，跳过")
                return True, f"节点 {node_name} 不存在，跳过"

            ret = subprocess.call(["rosnode", "kill", node_name], timeout=10)

            if ret == 0:
                self._log_info(f"rosnode kill {node_name} 成功")
                return True, f"已终止节点 {node_name}"
            else:
                self._log_error(f"rosnode kill {node_name} 失败，返回码: {ret}")
                return False, f"终止节点失败，返回码: {ret}"

        except subprocess.TimeoutExpired:
            return False, f"终止节点 {node_name} 超时"
        except Exception as e:
            self._log_error(f"终止节点 {node_name} 异常: {e}")
            return False, str(e)

    def cleanup_all(self) -> None:
        """清理所有由本服务启动的进程"""
        self._log_info("开始清理所有进程..."))

        for key in list(self._processes.keys()):
            self.kill_process(key)

        self._log_info("清理完成")

    def get_process_state(self, key: str) -> ProcessState:
        """获取指定进程的状态"""
        if key not in self._processes:
            return ProcessState.UNKNOWN

        proc = self._processes[key].process
        if proc.poll() is None:
            return ProcessState.RUNNING
        else:
            return ProcessState.STOPPED


class PackageManager(BaseManager):
    """包管理器，查询 ROS 包及其可执行文件"""

    def __init__(self):
        """初始化包管理器"""
        self._packages: Dict[str, str] = {}
        self._refresh_packages()
        self._log_info(f"包管理器初始化完成，发现 {len(self._packages)} 个包")

    def _refresh_packages(self) -> None:
        """使用 rospack list 刷新 ROS 包列表"""
        try:
            result = subprocess.run(["rospack", "list"], capture_output=True, text=True, timeout=30)

            self._packages.clear()
            for line in result.stdout.strip().split("\n"):
                if line:
                    parts = line.split(maxsplit=1)
                    if len(parts) == 2:
                        self._packages[parts[0]] = parts[1]

        except Exception as e:
            self._log_error(f"获取 ROS 包列表失败: {e}")

    @property
    def packages(self) -> Dict[str, str]:
        """获取所有 ROS 包，返回 {包名: 路径} 字典"""
        return self._packages.copy()

    def get_package_path(self, package_name: str) -> Optional[str]:
        """获取指定包的路径"""
        return self._packages.get(package_name)

    def list_executables(self, package_name: str) -> List[str]:
        """列出指定包中的所有可执行文件（含 .py 和 .launch）"""
        if package_name not in self._packages:
            self._log_warn(f"未找到包: {package_name}")
            return []

        package_path = self._packages[package_name]
        executables = set()

        try:
            cmd_exec = ["find", package_path, "-type", "f", "!", "-name", "*.*", "-executable"]
            result = subprocess.run(cmd_exec, capture_output=True, text=True, timeout=30)
            for line in result.stdout.strip().split("\n"):
                if line:
                    executables.add(os.path.basename(line))

            cmd_py_launch = [
                "find",
                package_path,
                "-type",
                "f",
                "(",
                "-iname",
                "*.py",
                "-executable",
                "-o",
                "-iname",
                "*.launch",
                ")",
            ]
            result = subprocess.run(cmd_py_launch, capture_output=True, text=True, timeout=30)
            for line in result.stdout.strip().split("\n"):
                if line:
                    executables.add(os.path.basename(line))

        except Exception as e:
            self._log_error(f"查找可执行文件失败: {e}")

        return sorted(executables)


class MapManager(BaseManager):
    """地图管理器，负责地图加载、保存和导航/建图模式切换"""

    def __init__(self, process_manager: ProcessManager, package_name: str = "seed_core"):
        """初始化地图管理器"""
        self._process_manager = process_manager
        self._package_name = package_name
        self._log_info("地图管理器初始化完成")

    def load_map(self, map_path: str) -> Tuple[bool, str]:
        """加载指定地图并重启导航"""
        expanded_path = os.path.expanduser(map_path)
        self._log_info(f"请求加载地图: {expanded_path}")
        self._stop_navigation_processes()
        rospy.sleep(2.0)
        os.environ["MAP"] = expanded_path
        self._log_info(f"已设置 MAP 环境变量: {expanded_path}")

        success, message = self._process_manager.start_launch(
            LaunchFile.NAVIGATION, ["roslaunch", self._package_name, LaunchFile.NAVIGATION]
        )

        if success:
            self._log_info(f"地图加载成功: {expanded_path}")
            return True, f"地图加载成功: {expanded_path}"
        else:
            self._log_error(f"启动导航失败: {message}")
            return False, f"启动导航失败: {message}"

    def save_map(self, file_path: str, topic: str = "/map") -> Tuple[bool, str]:
        """保存当前地图到指定路径（生成 map.yaml 和 map.pgm）"""
        expanded = os.path.expanduser(file_path)
        save_dir = os.path.abspath(expanded)
        self._log_info(f"请求保存地图到: {save_dir}")

        if not os.path.exists(save_dir):
            os.makedirs(save_dir, exist_ok=True)
            self._log_info(f"创建目录: {save_dir}")

        map_prefix = os.path.join(save_dir, "map")
        cmd = [
            "rosrun",
            "map_server",
            "map_saver",
            "-f",
            map_prefix,
            f"map:={topic}",
            "__name:=seed_core_map_saver",
        ]

        self._log_info(f"执行命令: {' '.join(cmd)}")

        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            flags = fcntl.fcntl(process.stdout, fcntl.F_GETFL)
            fcntl.fcntl(process.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)

            output_lines = []
            while True:
                if process.poll() is not None:
                    break
                try:
                    line = process.stdout.readline()
                    if not line:
                        break
                    decoded = line.decode("utf-8", "ignore").strip()
                    output_lines.append(decoded)
                    self._log_info(f"[map_saver] {decoded}")

                    if "[ERROR]" in decoded:
                        process.terminate()
                        return False, f"地图保存失败: {decoded}"
                except OSError:
                    break
                rospy.sleep(0.2)

            if process.returncode != 0:
                return False, f"map_saver 进程返回码: {process.returncode}"

            self._log_info("地图保存成功")
            return True, "地图保存成功"

        except Exception as e:
            self._log_error(f"保存地图异常: {e}")
            return False, str(e)

    def list_maps(self, directory: str) -> List[str]:
        """列出指定目录下包含 map.yaml 的子目录"""
        expanded = os.path.expanduser(directory)

        if not os.path.exists(expanded):
            self._log_warn(f"目录不存在: {expanded}")
            return []

        map_names = []
        try:
            for entry in os.listdir(expanded):
                subdir = os.path.join(expanded, entry)
                if os.path.isdir(subdir):
                    map_yaml = os.path.join(subdir, "map.yaml")
                    if os.path.isfile(map_yaml):
                        map_names.append(entry)

            map_names.sort()
            self._log_info(f"在 {expanded} 中找到 {len(map_names)} 个地图")

        except Exception as e:
            self._log_error(f"列出地图失败: {e}")

        return map_names

    def switch_to_navigation(self) -> Tuple[bool, str]:
        """切换到导航模式"""
        self._log_info("切换到导航模式")
        self._process_manager.kill_process(LaunchFile.MAPPING)
        rospy.sleep(2.0)

        success, message = self._process_manager.start_launch(
            LaunchFile.NAVIGATION, ["roslaunch", self._package_name, LaunchFile.NAVIGATION]
        )

        if success:
            return True, "已切换到导航模式"
        else:
            return False, f"切换到导航模式失败: {message}"

    def switch_to_mapping(self) -> Tuple[bool, str]:
        """切换到建图模式"""
        self._log_info("切换到建图模式")
        self._stop_navigation_processes()
        rospy.sleep(2.0)

        success, message = self._process_manager.start_launch(
            LaunchFile.MAPPING, ["roslaunch", self._package_name, LaunchFile.MAPPING]
        )

        if success:
            return True, "已切换到建图模式"
        else:
            return False, f"切换到建图模式失败: {message}"

    def _stop_navigation_processes(self) -> None:
        """停止导航相关的所有进程"""
        self._process_manager.kill_ros_node(NodeName.MOVE_BASE)
        self._process_manager.kill_ros_node(NodeName.SLAM_TOOLBOX)
        self._process_manager.kill_process(LaunchFile.NAVIGATION)


class FileOperationManager(BaseManager):
    """文件操作管理器，负责 PGM、JSON、文本文件的读写"""

    def read_pgm(self, file_path: str) -> Tuple[bool, str, Optional[Dict[str, Any]]]:
        """读取 PGM 格式的地图文件"""
        expanded = os.path.expanduser(file_path)

        if not os.path.exists(expanded):
            return False, f"文件不存在: {expanded}", None

        if not expanded.lower().endswith(".pgm"):
            return False, "文件必须是 .pgm 格式", None

        try:
            with Image.open(expanded) as img:
                img_array = np.array(img)
                height, width = img_array.shape
                max_val = int(img_array.max())

                data = {
                    "width": width,
                    "height": height,
                    "max_val": max_val,
                    "data": img_array.flatten().tolist(),
                }

                self._log_info(f"读取 PGM 文件成功: {width}x{height}")
                return True, f"读取成功: {width}x{height}", data

        except Exception as e:
            self._log_error(f"读取 PGM 文件失败: {e}")
            return False, str(e), None

    def update_pgm(
        self, file_path: str, width: int, height: int, data: List[int]
    ) -> Tuple[bool, str]:
        """更新或创建 PGM 文件"""
        expanded = os.path.expanduser(file_path)

        if width <= 0 or height <= 0:
            return False, "无效的图像尺寸"

        expected_length = width * height
        if len(data) != expected_length:
            return False, f"数据长度不匹配，期望 {expected_length}，实际 {len(data)}"

        try:
            directory = os.path.dirname(expanded)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)

            img_array = np.array(data, dtype=np.uint8).reshape((height, width))
            img = Image.fromarray(img_array, mode="L")
            img.save(expanded)

            self._log_info(f"更新 PGM 文件成功: {expanded}")
            return True, f"PGM 文件更新成功: {width}x{height}"

        except Exception as e:
            self._log_error(f"更新 PGM 文件失败: {e}")
            return False, str(e)

    def update_json(self, file_path: str, json_content: str) -> Tuple[bool, str]:
        """更新或创建 JSON 文件，自动创建备份"""
        expanded = os.path.expanduser(file_path)

        try:
            json_data = json.loads(json_content)
        except json.JSONDecodeError as e:
            return False, f"无效的 JSON 格式: {e}"

        try:
            directory = os.path.dirname(expanded)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)

            if os.path.exists(expanded):
                import shutil
                backup_path = f"{expanded}.backup.{int(rospy.Time.now().to_sec())}"
                shutil.copy2(expanded, backup_path)
                self._log_info(f"创建备份: {backup_path}")

            with open(expanded, "w", encoding="utf-8") as f:
                json.dump(json_data, f, indent=2, ensure_ascii=False)

            self._log_info(f"更新 JSON 文件成功: {expanded}")
            return True, "JSON 文件更新成功"

        except Exception as e:
            self._log_error(f"更新 JSON 文件失败: {e}")
            return False, str(e)

    def read_text_file(self, file_path: str) -> Tuple[bool, str, str]:
        """读取文本文件内容（支持 UTF-8 和 Latin-1）"""
        expanded = os.path.expanduser(file_path)

        if not os.path.exists(expanded):
            return False, f"文件不存在: {expanded}", ""

        if not os.path.isfile(expanded):
            return False, "路径不是文件", ""

        for encoding in ["utf-8", "latin-1"]:
            try:
                with open(expanded, encoding=encoding) as f:
                    content = f.read()
                self._log_info(f"读取文件成功 ({encoding}): {expanded}")
                return True, "读取成功", content
            except UnicodeDecodeError:
                continue

        return False, "无法解码文件内容", ""


class ServiceHandler:
    """服务处理器主类，整合所有管理器并注册 ROS 服务"""

    def __init__(self):
        """初始化服务处理器"""
        rospy.init_node("seed_core_service_handler_node")
        rospy.loginfo("=== Seed Core 服务处理器启动 ===")
        self._process_manager = ProcessManager()
        self._package_manager = PackageManager()
        self._map_manager = MapManager(self._process_manager, "seed_core")
        self._file_manager = FileOperationManager()
        self._start_initial_navigation()
        self._register_services()
        rospy.loginfo("=== 服务处理器就绪 ===")

    def _start_initial_navigation(self) -> None:
        """启动初始导航模式"""
        try:
            success, msg = self._process_manager.start_launch(
                LaunchFile.NAVIGATION, ["roslaunch", "seed_core", LaunchFile.NAVIGATION]
            )
            if success:
                rospy.loginfo("初始导航模式启动成功")
            else:
                rospy.logwarn(f"初始导航模式启动失败: {msg}")
        except Exception as e:
            rospy.logerr(f"启动初始导航模式异常: {e}")

    def _register_services(self) -> None:
        """注册所有 ROS 服务"""
        rospy.Service("seed_core/list_packages", ListPackages, self._handle_list_packages)
        rospy.Service("seed_core/list_executables", ListExecutables, self._handle_list_executables)
        rospy.Service("seed_core/node/start", ManageNode, self._handle_node_start)
        rospy.Service("seed_core/node/kill", ManageNode, self._handle_node_kill)
        rospy.Service("seed_core/node/info", ManageNode, self._handle_node_info)
        rospy.Service("seed_core/roswtf", Trigger, self._handle_roswtf)
        rospy.Service("seed_core/load_map", LoadMap, self._handle_load_map)
        rospy.Service("seed_core/save_map", SaveMap, self._handle_save_map)
        rospy.Service("seed_core/list_maps", ListMaps, self._handle_list_maps)
        rospy.Service("seed_core/switch_to_navigation", Trigger, self._handle_switch_to_navigation)
        rospy.Service("seed_core/switch_to_mapping", Trigger, self._handle_switch_to_mapping)
        rospy.Service("seed_core/read_pgm", ReadPgm, self._handle_read_pgm)
        rospy.Service("seed_core/update_pgm", UpdatePgm, self._handle_update_pgm)
        rospy.Service("seed_core/update_json", UpdateJson, self._handle_update_json)
        rospy.Service("seed_core/read_text_file", ReadTextFile, self._handle_read_text_file)

    def _handle_list_packages(self, req) -> ListPackagesResponse:
        """处理列出包服务请求"""
        return ListPackagesResponse(list(self._package_manager.packages.keys()))

    def _handle_list_executables(self, req) -> ListExecutablesResponse:
        """处理列出可执行文件服务请求"""
        executables = self._package_manager.list_executables(req.package)
        return ListExecutablesResponse(executables)

    def _handle_node_start(self, req) -> ManageNodeResponse:
        """处理启动节点服务请求"""
        try:
            args = shlex.split(req.node)

            if args and args[0] == "roslaunch":
                launch_files = [arg for arg in args if arg.endswith(".launch")]
                key = launch_files[0] if launch_files else args[-1]
            else:
                key = args[-1]

            success, message = self._process_manager.start_launch(key, args)
            return ManageNodeResponse(success=success, message=message)

        except Exception as e:
            rospy.logerr(f"启动节点失败: {e}")
            return ManageNodeResponse(success=False, message=str(e))

    def _handle_node_kill(self, req) -> ManageNodeResponse:
        """处理终止节点服务请求"""
        try:
            key = req.node.strip()

            if ".launch" in key:
                launch_file = next((tok for tok in key.split() if tok.endswith(".launch")), key)
                success, message = self._process_manager.kill_process(launch_file)
            else:
                success, message = self._process_manager.kill_ros_node(key)

            return ManageNodeResponse(success=success, message=message)

        except Exception as e:
            rospy.logerr(f"终止节点失败: {e}")
            return ManageNodeResponse(success=False, message=str(e))

    def _handle_node_info(self, req) -> ManageNodeResponse:
        """处理查询节点信息服务请求"""
        try:
            result = subprocess.check_output(["rosnode", "info", req.node], timeout=10).decode(
                "utf-8"
            )
            # 清理输出格式
            result = result.replace(
                "--------------------------------------------------------------------------------",
                "",
            )
            return ManageNodeResponse(success=True, message=result)
        except Exception as e:
            return ManageNodeResponse(success=False, message=str(e))

    def _handle_roswtf(self, req) -> TriggerResponse:
        """处理 roswtf 诊断服务请求"""
        try:
            result = subprocess.check_output(["roswtf"], timeout=60).decode("utf-8")
            return TriggerResponse(success=True, message=result)
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def _handle_load_map(self, req) -> LoadMapResponse:
        """处理加载地图服务请求"""
        success, message = self._map_manager.load_map(req.file_path)
        return LoadMapResponse(success=success, message=message)

    def _handle_save_map(self, req) -> SaveMapResponse:
        """处理保存地图服务请求"""
        topic = req.topic if req.topic else "/map"
        success, message = self._map_manager.save_map(req.file_path, topic)
        return SaveMapResponse(success=success, message=message)

    def _handle_list_maps(self, req) -> ListMapsResponse:
        """处理列出地图服务请求"""
        maps = self._map_manager.list_maps(req.directory)
        return ListMapsResponse(maps=maps)

    def _handle_switch_to_navigation(self, req) -> TriggerResponse:
        """处理切换到导航模式服务请求"""
        success, message = self._map_manager.switch_to_navigation()
        return TriggerResponse(success=success, message=message)

    def _handle_switch_to_mapping(self, req) -> TriggerResponse:
        """处理切换到建图模式服务请求"""
        success, message = self._map_manager.switch_to_mapping()
        return TriggerResponse(success=success, message=message)

    def _handle_read_pgm(self, req) -> ReadPgmResponse:
        """处理读取 PGM 文件服务请求"""
        success, message, data = self._file_manager.read_pgm(req.file_path)

        if success and data:
            return ReadPgmResponse(
                success=True,
                message=message,
                width=data["width"],
                height=data["height"],
                max_val=data["max_val"],
                data=data["data"],
            )
        else:
            return ReadPgmResponse(success=False, message=message)

    def _handle_update_pgm(self, req) -> UpdatePgmResponse:
        """处理更新 PGM 文件服务请求"""
        success, message = self._file_manager.update_pgm(
            req.file_path, req.width, req.height, list(req.data)
        )
        return UpdatePgmResponse(success=success, message=message)

    def _handle_update_json(self, req) -> UpdateJsonResponse:
        """处理更新 JSON 文件服务请求"""
        success, message = self._file_manager.update_json(req.file_path, req.json_content)
        return UpdateJsonResponse(success=success, message=message)

    def _handle_read_text_file(self, req) -> ReadTextFileResponse:
        """处理读取文本文件服务请求"""
        success, message, content = self._file_manager.read_text_file(req.file_path)
        return ReadTextFileResponse(success=success, message=message, content=content)

    def cleanup(self) -> None:
        """清理所有资源"""
        rospy.loginfo("开始清理服务处理器...")
        self._process_manager.cleanup_all()
        rospy.loginfo("服务处理器清理完成")

    def signal_handler(self, signum, frame) -> None:
        """系统信号处理器，确保优雅退出"""
        rospy.loginfo(f"收到信号 {signum}，开始清理...")
        self.cleanup()
        sys.exit(0)


if __name__ == "__main__":
    try:
        handler = ServiceHandler()
        signal.signal(signal.SIGINT, handler.signal_handler)
        signal.signal(signal.SIGTERM, handler.signal_handler)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("服务处理器已关闭")
    except Exception as e:
        rospy.logerr(f"服务处理器异常: {e}")
        raise
