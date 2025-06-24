import { reactive, markRaw, computed } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { MapController } from './map.controller'

export interface MapState {
  rosBridgeServerUrl: string // ROS Bridge 服务器的 WebSocket URL
  gridConfig: {
    size: number // 主网格线的间距（地图单位，米）
    thickness: number // 主网格线的线宽（像素）
    colour: string // 主网格线颜色
    colour_sub: string // 子网格线颜色
    autoscale: 'Off' | 'Very Fine' | 'Fine' | 'Coarse' | 'Rough' // 网格自适应级别
    subdivisions: number // 每个主网格的细分数量
    maxGridLines: number // 最大网格线数量限制
    subGridOpacity: number // 子网格线透明度
  }
}

export const useMapStore = defineStore('map', () => {
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
  })

  const controller = markRaw(new MapController(state))

  return {
    state,
    controller,
  }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useMapStore, import.meta.hot))
}
