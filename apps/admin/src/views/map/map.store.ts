import { reactive, markRaw, computed } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { createLogger } from '@/core/logger.service'
import { MapController } from './map.controller'
import { DefaultViewManagerFactory, DefaultGridManagerFactory } from './map.service'
import type { MapState, MapDeps } from './map.types'

/**
 * 创建 Map 模块的依赖
 *
 * 所有外部依赖在此组装：
 * - logger: 日志服务
 * - viewManagerFactory: ViewManager 工厂
 * - gridManagerFactory: GridManager 工厂
 * - apiService: API 服务（可选）
 */
function createMapDeps(): MapDeps {
  return {
    logger: createLogger('Map'),
    viewManagerFactory: new DefaultViewManagerFactory(),
    gridManagerFactory: new DefaultGridManagerFactory(),
    // apiService: new HttpMapApiService('/api'), // 可选
  }
}

/**
 * Map 状态管理
 *
 * 采用 Controller + Store 分层架构：
 * - Store: 负责响应式状态托管、计算属性
 * - Controller: 负责核心业务逻辑（继承 BaseController）
 *
 * 依赖注入：
 * - 通过 Deps 注入 ViewManagerFactory 和 GridManagerFactory
 * - 便于测试时替换为 Mock 实现
 */
export const useMapStore = defineStore('map', () => {
  // 依赖
  const deps = createMapDeps()

  // 状态（响应式）
  const state = reactive<MapState>({
    rosBridgeServerUrl: 'ws://10.211.55.5:5001',
    gridConfig: {
      size: 1,
      thickness: 1,
      colour: '#3e556a',
      colour_sub: '#294056',
      autoscale: 'Coarse',
      subdivisions: 2,
      maxGridLines: 300,
      subGridOpacity: 0.65,
    },
    loading: false,
    errorMessage: null,
    viewInitialized: false,
    gridInitialized: false,
  })

  // Controller（使用 markRaw 避免响应式包装）
  const controller = markRaw(new MapController(state, deps))

  // 计算属性
  const isViewReady = computed(() => state.viewInitialized)
  const isGridReady = computed(() => state.gridInitialized)
  const hasError = computed(() => !!state.errorMessage)

  return {
    state,
    controller,
    // Getters
    isViewReady,
    isGridReady,
    hasError,
  }
})

// 热模块替换支持
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useMapStore, import.meta.hot))
}
