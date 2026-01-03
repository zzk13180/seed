import { reactive, shallowRef, computed } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { createLogger } from '@/core/logger.service'
import { errorHandler } from '@/core/error.service'
import { PanelController } from './panel.controller'
import { MockPanelApiService } from './panel.service'
import type { PanelState, PanelEnv } from './panel.types'
import type { PanelController as PanelControllerType } from './panel.controller'

// 组装环境依赖
const env: PanelEnv = {
  logger: createLogger('Panel'),
  apiService: new MockPanelApiService(),
  // wsService: new WebSocketPanelService(), // 可选
  errorHandler,
}

// HMR 替换函数（仅开发环境使用）
let _hotReplaceController: ((NewClass: typeof PanelController) => void) | null = null

/**
 * Panel 状态管理
 *
 * 采用 Controller + Store 分层架构：
 * - Store: 负责响应式状态托管、计算属性
 * - Controller: 负责核心业务逻辑（继承 BaseController）
 *
 * 使用 shallowRef 支持 Controller HMR 热更新
 */
export const usePanelStore = defineStore('panel', () => {
  // 状态（响应式）
  const state = reactive<PanelState>({
    robotInfo: {},
    loading: false,
    errorMessage: null,
    wsConnected: false,
  })

  // Controller（使用 shallowRef 支持 HMR 替换）
  const controllerRef = shallowRef(new PanelController(state, env))

  // 计算属性（便于模板绑定）
  const robotId = computed(() => controllerRef.value.robotId)
  const robotStatus = computed(() => controllerRef.value.robotStatus)
  const isLoading = computed(() => state.loading)
  const hasError = computed(() => !!state.errorMessage)

  // 注册 HMR 替换逻辑
  if (import.meta.hot) {
    _hotReplaceController = (NewControllerClass: typeof PanelController) => {
      void controllerRef.value.dispose() // 清理旧实例
      controllerRef.value = new NewControllerClass(state, env) // 注入相同的 state 和 env
      env.logger.info('PanelController instance has been updated via HMR.')
    }
  }

  return {
    state,
    controller: controllerRef,
    // Getters
    robotId,
    robotStatus,
    isLoading,
    hasError,
  }
})

// 热模块替换支持
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(usePanelStore, import.meta.hot))
  import.meta.hot.accept(
    './panel.controller',
    (newModule: { PanelController?: typeof PanelControllerType } | undefined) => {
      if (newModule?.PanelController && _hotReplaceController) {
        _hotReplaceController(newModule.PanelController)
      } else {
        console.error('HMR update for PanelController failed.')
      }
    },
  )
}
