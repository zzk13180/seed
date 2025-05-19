import { reactive, computed, markRaw } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { PanelController } from './panel.controller'

export interface RobotInfo {
  robotId?: string
}

export interface LogEntry {
  time: string
  message: string
  type: 'info' | 'success' | 'error' | 'warning'
}

export interface State {
  robotInfo: RobotInfo
  logs: LogEntry[]
  connectionStatus: string
  pkgName: string
  nodeCmd: string
  nodeNameInfo: string
  serviceResponse: string
  tfConsolidatedData: string
}

export const usePanelStore = defineStore('panel', () => {
  const state: State = reactive({
    robotInfo: {},
    logs: [],
    connectionStatus: '正在连接到ROS服务器...',
    pkgName: 'turtlesim',
    nodeCmd: 'rosrun turtlesim turtlesim_node',
    nodeNameInfo: '/turtlesim',
    serviceResponse: '等待操作...',
    tfConsolidatedData: '等待数据...',
  })

  const controller = markRaw(new PanelController(state))
  const getRobotId = computed(() => controller.getRobotId)

  return {
    state,
    controller,
    getter: {
      getRobotId,
    },
  }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(usePanelStore, import.meta.hot))
}
