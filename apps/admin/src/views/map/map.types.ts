/**
 * Map 模块的类型定义
 * 纯 TypeScript 类型，不依赖任何框架
 *
 * 架构说明：
 * - 使用工厂模式创建 ViewManager 和 GridManager
 * - 通过 Env 依赖注入，支持测试时替换实现
 * - ViewManager/GridManager 提供抽象接口，隐藏实现细节
 */

import type { Logger } from '@/core/logger.service'
import type { Observable } from 'rxjs'

/** 网格自适应级别 */
export type GridAutoscale = 'Off' | 'Very Fine' | 'Fine' | 'Coarse' | 'Rough'

/** 2D 坐标点 */
export interface Point2D {
  x: number
  y: number
}

/** 3D 坐标点 */
export interface Point3D extends Point2D {
  z: number
}

/** 视图状态 */
export interface ViewState {
  /** 视图中心点坐标（地图坐标系） */
  viewCenter: Point2D
  /** 视图缩放级别（数值越大，显示越大） */
  viewScale: number
  /** 是否允许用户移动/缩放地图 */
  inputMovement: boolean
}

/** 网格配置 */
export interface GridConfig {
  /** 主网格线的间距（地图单位，米） */
  size: number
  /** 主网格线的线宽（像素） */
  thickness: number
  /** 主网格线颜色 */
  colour: string
  /** 子网格线颜色 */
  colour_sub: string
  /** 网格自适应级别 */
  autoscale: GridAutoscale
  /** 每个主网格的细分数量 */
  subdivisions: number
  /** 最大网格线数量限制 */
  maxGridLines: number
  /** 子网格线透明度 */
  subGridOpacity: number
}

/**
 * 视图管理器接口
 * 负责管理地图视图的中心位置和缩放级别
 */
export interface IViewManager {
  /** 视图状态变化的 Observable 流 */
  readonly viewStateChange$: Observable<ViewState>

  /** 容器宽度 */
  readonly containerWidth: number

  /** 容器高度 */
  readonly containerHeight: number

  /** 获取当前视图状态 */
  getState(): ViewState

  /** 设置视图状态 */
  setState(state: ViewState): void

  /** 添加事件监听器 */
  addListeners(): void

  /** 移除事件监听器 */
  removeListeners(): void

  /** 设置是否允许输入移动 */
  setInputMovementEnabled(value: boolean): void

  /** 重置视图 */
  resetView(defaultCenter?: Point2D, defaultScale?: number): void

  /** 地图坐标 → 屏幕坐标 */
  fixedToScreen(coords: Point2D): Point3D

  /** 屏幕坐标 → 地图坐标 */
  screenToFixed(coords: Point2D): Point3D

  /** 像素长度 → 地图单位长度 */
  getPixelsInMapUnits(length: number): number

  /** 地图单位长度 → 像素长度 */
  getMapUnitsInPixels(length: number): number

  /** 销毁实例 */
  destroy(): void
}

/**
 * ViewManager 配置选项
 */
export interface ViewManagerOptions {
  /** 判断是否应阻止对给定目标元素的拖动操作 */
  shouldPreventDrag?: (target: EventTarget | null) => boolean
}

/**
 * ViewManager 工厂接口
 * 通过依赖注入提供，支持测试时替换实现
 */
export interface IViewManagerFactory {
  create(container: HTMLElement, options?: ViewManagerOptions): IViewManager
}

/**
 * 网格事件回调
 */
export interface GridEvents {
  onResize?: () => void
}

/**
 * 网格管理器接口
 * 负责绘制和管理地图网格
 */
export interface IGridManager {
  /** 初始化网格 */
  initialize(): void

  /** 销毁网格管理器 */
  destroy(): void

  /** 手动触发重绘 */
  draw?(): void
}

/**
 * GridManager 工厂接口
 * 通过依赖注入提供，支持测试时替换实现
 */
export interface IGridManagerFactory {
  create(
    mapState: MapState,
    canvas: HTMLCanvasElement,
    viewManager: IViewManager,
    events?: GridEvents,
  ): IGridManager
}

/** Map 模块状态 */
export interface MapState {
  /** ROS Bridge 服务器的 WebSocket URL */
  rosBridgeServerUrl: string
  /** 网格配置 */
  gridConfig: GridConfig
  /** 是否正在加载 */
  loading: boolean
  /** 错误信息 */
  errorMessage: string | null
  /** 视图管理器是否已初始化 */
  viewInitialized: boolean
  /** 网格管理器是否已初始化 */
  gridInitialized: boolean
}

/**
 * Map API 服务接口
 */
export interface MapApiService {
  fetchMapData(): Promise<unknown>
  saveMapConfig(config: GridConfig): Promise<void>
}

/**
 * Map 模块环境依赖
 *
 * 所有外部依赖通过 Env 注入，便于测试和替换实现
 */
export interface MapEnv {
  /** 日志服务 */
  logger: Logger

  /** API 服务（可选） */
  apiService?: MapApiService

  /** ViewManager 工厂 */
  viewManagerFactory: IViewManagerFactory

  /** GridManager 工厂 */
  gridManagerFactory: IGridManagerFactory
}
