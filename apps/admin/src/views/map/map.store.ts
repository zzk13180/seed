import { reactive, markRaw, computed } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { MapController } from './map.controller'

export interface MapState {
  rosBridgeServerUrl: string // ROS Bridge 服务器的 WebSocket URL
  viewCenter: { x: number; y: number } // 当前视图中心点的地图坐标
  viewScale: number // 当前视图缩放比例（地图单位到屏幕像素的比例）
  inputMovement: boolean // 是否允许通过鼠标/触摸拖动地图
  gridConfig: {
    size: number // 主网格线的间距（地图单位，米）
    thickness: number // 主网格线的线宽（像素）
    colour: string // 主网格线颜色
    colour_sub: string // 子网格线颜色
    autoscale: 'Off' | 'Very Fine' | 'Fine' | 'Coarse' | 'Rough' // 网格自适应级别
    subdivisions: number // 每个主网格的细分数量
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

  return {
    state,
    controller,
    getter: {
      computedExample,
    },
  }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useMapStore, import.meta.hot))
}
