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
}

export const usePanelStore = defineStore('panel', () => {
  const state: State = reactive({
    robotInfo: {},
    logs: [],
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
  // import.meta.hot.accept('./panel.controller', newPanelControllerModule => {
  //   if (newPanelControllerModule && newPanelControllerModule.PanelController) {
  //     const store = usePanelStore() // 获取当前 store 实例
  //     // eslint-disable-next-line @typescript-eslint/no-unsafe-assignment
  //     const newController = new newPanelControllerModule.PanelController(store.state)
  //     // eslint-disable-next-line @typescript-eslint/no-unsafe-assignment
  //     store.controller = newController
  //     console.log('PanelController instance has been updated via HMR.')
  //   } else {
  //     console.error(
  //       'HMR update for PanelController failed: New module or PanelController class not found.',
  //     )
  //   }
  // })
}
