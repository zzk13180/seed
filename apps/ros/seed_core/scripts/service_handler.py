#!/usr/bin/env python3

import os
import sys
import subprocess
import rospy

from seed_core.srv import ManageNode, ManageNodeResponse
from seed_core.srv import ListPackages, ListPackagesResponse
from seed_core.srv import ListExecutables, ListExecutablesResponse

class ServiceManager:
    """ROS服务管理类，负责节点的启动、关闭、信息查询及包/可执行文件的列举。"""

    def __init__(self):
        rospy.init_node('seed_core_service_handler')
        self.packages = self._get_packages()

        # 注册服务
        self._register_services()
        rospy.loginfo("ServiceManager 已就绪。")

    def _register_services(self):
        self.node_kill_service = rospy.Service(
            'seed_core/node/kill', ManageNode, self.handle_node_kill)
        self.node_start_service = rospy.Service(
            'seed_core/node/start', ManageNode, self.handle_node_start)
        self.node_info_service = rospy.Service(
            'seed_core/node/info', ManageNode, self.handle_node_info)
        self.list_packages_service = rospy.Service(
            'seed_core/list_packages', ListPackages, self.handle_list_packages)
        self.list_executables_service = rospy.Service(
            'seed_core/list_executables', ListExecutables, self.handle_list_executables)

    def _get_packages(self):
        """获取所有ROS包及其路径。"""
        try:
            output = subprocess.check_output(["rospack", "list"]).decode('utf-8')
            packages = {}
            for line in output.strip().split('\n'):
                if line:
                    name, path = line.split(maxsplit=1)
                    packages[name] = path
            return packages
        except Exception as e:
            rospy.logerr(f"获取ROS包失败: {e}")
            return {}

    def handle_list_packages(self, req):
        """返回所有包名。"""
        return ListPackagesResponse(list(self.packages.keys()))

    def handle_list_executables(self, req):
        """返回指定包下的可执行文件、.py和.launch文件。"""
        package = req.package
        if package not in self.packages:
            rospy.logerr(f"未找到包: {package}")
            return ListExecutablesResponse([])

        path = self.packages[package]
        executables = self._find_executables(path)
        return ListExecutablesResponse(executables)

    def _find_executables(self, path):
        """查找可执行文件、.py和.launch文件。"""
        result = set()
        try:
            # 查找无扩展名可执行文件
            cmd_exec = ["find", path, "-type", "f", "!", "-name", "*.*", "-executable"]
            exec_files = subprocess.check_output(cmd_exec).decode('utf-8').splitlines()
            result.update(os.path.basename(f) for f in exec_files if f)

            # 查找可执行.py和所有.launch文件
            cmd_py_launch = [
                "find", path, "-type", "f",
                "(", "-iname", "*.py", "-executable", "-o", "-iname", "*.launch", ")"
            ]
            py_launch_files = subprocess.check_output(cmd_py_launch).decode('utf-8').splitlines()
            result.update(os.path.basename(f) for f in py_launch_files if f)
        except Exception as e:
            rospy.logerr(f"查找可执行文件失败: {e}")
        return sorted(result)

    def handle_node_kill(self, req):
        """关闭指定节点。"""
        try:
            subprocess.check_call(['rosnode', 'kill', req.node])
            return ManageNodeResponse(success=True, message=f"已关闭节点 {req.node}")
        except Exception as e:
            return ManageNodeResponse(success=False, message=str(e))

    def handle_node_start(self, req):
        """启动指定节点。"""
        try:
            args = req.node.split()
            devnull = open(os.devnull, 'w')

            def preexec():
                os.setpgrp()
                sys.stdin = open(os.devnull, 'r')
                sys.stdout = open(os.devnull, 'w')
                sys.stderr = open(os.devnull, 'w')

            subprocess.Popen(args, stdout=devnull, stderr=devnull, preexec_fn=preexec)
            return ManageNodeResponse(success=True, message=f"已启动节点 {req.node}")
        except Exception as e:
            return ManageNodeResponse(success=False, message=str(e))

    def handle_node_info(self, req):
        """查询节点信息。"""
        try:
            info = subprocess.check_output(["rosnode", "info", req.node]).decode('utf-8')
            info = info.replace("--------------------------------------------------------------------------------", "")
            return ManageNodeResponse(success=True, message=info)
        except Exception as e:
            return ManageNodeResponse(success=False, message=str(e))


if __name__ == "__main__":
    manager = ServiceManager()
    rospy.spin()
