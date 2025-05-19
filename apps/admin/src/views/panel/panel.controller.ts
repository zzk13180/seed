/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/* eslint-disable @typescript-eslint/no-unsafe-member-access */
import * as ROSLIB from 'roslib'
import type { State, LogEntry } from './panel.store'

export class PanelController {
  private state: State
  private ros: any // ROS连接对象

  constructor(state: State) {
    this.state = state
  }

  addLog(message: string, type: LogEntry['type'] = 'info') {
    const now = new Date()
    const timeStr = now.toLocaleTimeString()
    this.state.logs.unshift({
      time: timeStr,
      message,
      type,
    })
    if (this.state.logs.length > 50) {
      this.state.logs = this.state.logs.slice(0, 50)
    }
  }

  clearLogs() {
    this.state.logs = []
  }

  // 组件挂载时建立ROS连接
  connectToRos() {
    // 创建与ROS桥接服务器的连接
    this.ros = new ROSLIB.Ros({
      url: 'ws://10.211.55.5:5001', // ROS桥接服务器地址
    })

    // 连接成功事件处理
    this.ros.on('connection', () => {
      this.state.connectionStatus = '已连接到ROS服务器'
      this.subscribeToTfConsolidated() // 开始订阅TF话题
      this.addLog('已连接到ROS服务器', 'success')
    })

    // 连接错误事件处理
    this.ros.on('error', (error: any) => {
      this.state.connectionStatus = `连接错误: ${error}`
      console.error('连接错误: ', error)
      this.addLog(`ROS连接错误: ${error}`, 'error')
    })

    // 连接关闭事件处理
    this.ros.on('close', () => {
      this.state.connectionStatus = 'ROS服务器连接已关闭'
      this.addLog('ROS服务器连接已关闭', 'warning')
    })
  }

  // 显示服务调用响应
  displayServiceResponse(data: any) {
    this.state.serviceResponse = JSON.stringify(data, null, 2)
  }

  // 列出所有ROS包
  listPackages() {
    if (!this.ros) {
      this.addLog('尚未连接到ROS服务器', 'error')
      return
    }

    // 创建服务调用
    const service = new ROSLIB.Service({
      ros: this.ros,
      name: '/seed_core/list_packages',
      serviceType: 'seed_core/ListPackages',
    })

    // 调用服务
    service.callService(
      new ROSLIB.ServiceRequest({}),
      (result: any) => {
        this.displayServiceResponse(result)
        this.addLog('成功获取ROS包列表', 'success')
      },
      (error: any) => {
        this.displayServiceResponse({ error })
        this.addLog(`获取ROS包列表失败: ${error}`, 'error')
      },
    )
  }

  // 列出指定包中的可执行文件
  listExecutables() {
    if (!this.ros) {
      this.addLog('尚未连接到ROS服务器', 'error')
      return
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name: '/seed_core/list_executables',
      serviceType: 'seed_core/ListExecutables',
    })

    service.callService(
      new ROSLIB.ServiceRequest({ package: this.state.pkgName }),
      (result: any) => {
        this.displayServiceResponse(result)
        this.addLog(`成功获取包 ${this.state.pkgName} 的可执行文件列表`, 'success')
      },
      (error: any) => {
        this.displayServiceResponse({ error })
        this.addLog(`获取可执行文件列表失败: ${error}`, 'error')
      },
    )
  }

  // 通用节点管理服务调用函数
  callManageNodeService(serviceName: string, nodeValue: string) {
    if (!this.ros) {
      this.addLog('尚未连接到ROS服务器', 'error')
      return
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name: `/seed_core/node/${serviceName}`,
      serviceType: 'seed_core/ManageNode',
    })

    service.callService(
      new ROSLIB.ServiceRequest({ node: nodeValue }),
      (result: any) => {
        this.displayServiceResponse(result)
        this.addLog(`节点操作 ${serviceName} 成功`, 'success')
      },
      (error: any) => {
        this.displayServiceResponse({ error })
        this.addLog(`节点操作 ${serviceName} 失败: ${error}`, 'error')
      },
    )
  }

  // 启动节点
  startNode() {
    this.addLog(`尝试启动节点: ${this.state.nodeCmd}`, 'info')
    this.callManageNodeService('start', this.state.nodeCmd)
  }

  // 获取节点信息
  getNodeInfo() {
    this.addLog(`获取节点信息: ${this.state.nodeNameInfo}`, 'info')
    this.callManageNodeService('info', this.state.nodeNameInfo)
  }

  // 关闭节点
  killNode() {
    this.addLog(`关闭节点: ${this.state.nodeNameInfo}`, 'warning')
    this.callManageNodeService('kill', this.state.nodeNameInfo)
  }

  // 订阅TF合并话题
  subscribeToTfConsolidated() {
    if (!this.ros) {
      this.addLog('尚未连接到ROS服务器，无法订阅话题', 'error')
      return
    }

    const listener = new ROSLIB.Topic({
      ros: this.ros,
      name: '/seed_core/tf_consolidated',
      messageType: 'tf2_msgs/TFMessage',
    })

    listener.subscribe((message: any) => {
      this.state.tfConsolidatedData = JSON.stringify(message, null, 2)
    })

    this.addLog('已订阅 /seed_core/tf_consolidated 话题', 'info')
  }

  onComponentMounted() {
    this.addLog('机器人控制面板已初始化，正在连接到ROS服务器')
    this.connectToRos()
  }

  get getRobotId(): string {
    return this.state.robotInfo.robotId || ''
  }
}
