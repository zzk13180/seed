import { reactive, markRaw, computed } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { MapController } from './map.controller'

export interface MapState {
  rosBridgeServerUrl: string // ROS Bridge 服务器 URL
  viewCenter: { x: number; y: number }
  viewScale: number
  inputMovement: boolean
  viewChangeCallback?: (view: { center: { x: number; y: number }; scale: number }) => void
  // 网格配置
  gridConfig: {
    size: number
    thickness: number
    colour: string
    colour_sub: string
    autoscale: 'Off' | 'Very Fine' | 'Fine' | 'Coarse' | 'Rough'
    subdivisions: number
  }
}

export const useMapStore = defineStore('map', () => {
  const state = reactive<MapState>({
    rosBridgeServerUrl: 'ws://10.211.55.5:5001',
    viewCenter: { x: 0, y: 0 },
    viewScale: 50.0,
    inputMovement: true,
    gridConfig: {
      size: 1.0,
      thickness: 1,
      colour: '#3e556a',
      colour_sub: '#294056',
      autoscale: 'Coarse',
      subdivisions: 2,
    },
  })

  const controller = markRaw(new MapController(state))

  const computedExample = computed(() => controller.computedExample)

  // 公开视图控制功能
  const initializeMapView = (container: HTMLElement) => {
    controller.initialize(container)
  }

  const destroyMapView = () => {
    controller.destroy()
  }

  const setViewCenter = (center: { x: number; y: number }) => {
    controller.setView(center, undefined)
  }

  const setViewScale = (scale: number) => {
    controller.setView(undefined, scale)
  }

  const resetView = () => {
    controller.resetView()
  }

  // 网格相关方法
  const initializeGrid = (canvas: HTMLCanvasElement) => {
    controller.initializeGrid(canvas)
  }

  const updateGridConfig = (config: Partial<MapState['gridConfig']>) => {
    Object.assign(state.gridConfig, config)
    controller.drawGrid()
  }

  const resizeGridCanvas = () => {
    controller.resizeGridCanvas()
  }

  return {
    state,
    controller,
    initializeMapView,
    destroyMapView,
    setViewCenter,
    setViewScale,
    resetView,
    initializeGrid,
    updateGridConfig,
    resizeGridCanvas,
    getter: {
      computedExample,
    },
  }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useMapStore, import.meta.hot))
}
