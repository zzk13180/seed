/**
 * Map 模块服务实现
 *
 * 提供 ViewManager 和 GridManager 的工厂实现
 * 这些工厂通过 Env 注入到 Controller 中
 */

import { ViewManager } from './managers/view.manager'
import { MapGridManager } from './MapGridCanvas/map-grid-manager'
import type {
  IViewManager,
  IViewManagerFactory,
  IGridManager,
  IGridManagerFactory,
  ViewManagerOptions,
  MapState,
  GridEvents,
  MapApiService,
  GridConfig,
} from './map.types'

/**
 * 默认 ViewManager 工厂
 * 创建标准的 ViewManager 实例
 */
export class DefaultViewManagerFactory implements IViewManagerFactory {
  create(container: HTMLElement, options?: ViewManagerOptions): IViewManager {
    const viewManager = new ViewManager(container, options)

    // 适配器：确保返回的对象符合 IViewManager 接口
    return {
      get viewStateChange$() {
        return viewManager.viewStateChange$
      },
      get containerWidth() {
        return viewManager.getContainerWidth
      },
      get containerHeight() {
        return viewManager.getContainerHeight
      },
      getState: () => viewManager.getState(),
      setState: state => viewManager.setState(state),
      addListeners: () => viewManager.addListeners(),
      removeListeners: () => viewManager.removeListeners(),
      setInputMovementEnabled: value => viewManager.setInputMovementEnabled(value),
      resetView: (center?, scale?) => viewManager.resetView(center, scale),
      fixedToScreen: coords => viewManager.fixedToScreen(coords),
      screenToFixed: coords => viewManager.screenToFixed(coords),
      getPixelsInMapUnits: length => viewManager.getPixelsInMapUnits(length),
      getMapUnitsInPixels: length => viewManager.getMapUnitsInPixels(length),
      destroy: () => viewManager.destroy(),
    }
  }
}

/**
 * 默认 GridManager 工厂
 * 创建标准的 MapGridManager 实例
 */
export class DefaultGridManagerFactory implements IGridManagerFactory {
  create(
    mapState: MapState,
    canvas: HTMLCanvasElement,
    viewManager: IViewManager,
    events?: GridEvents,
  ): IGridManager {
    // 需要将 IViewManager 接口转换回 ViewManager
    // 因为 MapGridManager 目前依赖具体类型
    // 这里创建一个代理对象满足 MapGridManager 的需求
    const viewManagerProxy = {
      container: canvas.parentElement as HTMLElement,
      viewStateChange$: viewManager.viewStateChange$,
      get getContainerWidth() {
        return viewManager.containerWidth
      },
      get getContainerHeight() {
        return viewManager.containerHeight
      },
      getState: () => viewManager.getState(),
      setState: (state: unknown) =>
        viewManager.setState(state as ReturnType<typeof viewManager.getState>),
      addListeners: () => viewManager.addListeners(),
      removeListeners: () => viewManager.removeListeners(),
      setInputMovementEnabled: (value: boolean) => viewManager.setInputMovementEnabled(value),
      resetView: (center?: { x: number; y: number }, scale?: number) =>
        viewManager.resetView(center, scale),
      fixedToScreen: (coords: { x: number; y: number }) => viewManager.fixedToScreen(coords),
      screenToFixed: (coords: { x: number; y: number }) => viewManager.screenToFixed(coords),
      getPixelsInMapUnits: (length: number) => viewManager.getPixelsInMapUnits(length),
      getMapUnitsInPixels: (length: number) => viewManager.getMapUnitsInPixels(length),
      destroy: () => viewManager.destroy(),
    }

    const gridManager = new MapGridManager(
      mapState,
      canvas,
      viewManagerProxy as unknown as ConstructorParameters<typeof MapGridManager>[2],
      events,
    )

    return {
      initialize: () => gridManager.initialize(),
      destroy: () => gridManager.destroy(),
    }
  }
}

/**
 * Mock API 服务（开发/测试用）
 */
export class MockMapApiService implements MapApiService {
  async fetchMapData(): Promise<unknown> {
    // 模拟网络延迟
    await new Promise(resolve => setTimeout(resolve, 100))
    return {
      width: 100,
      height: 100,
      resolution: 0.05,
      origin: { x: -50, y: -50 },
    }
  }

  async saveMapConfig(_config: GridConfig): Promise<void> {
    // 模拟保存
    await new Promise(resolve => setTimeout(resolve, 100))
    console.log('Grid config saved (mock)')
  }
}

/**
 * HTTP API 服务
 */
export class HttpMapApiService implements MapApiService {
  constructor(private readonly baseUrl: string = '/api') {}

  async fetchMapData(): Promise<unknown> {
    const response = await fetch(`${this.baseUrl}/map`)
    if (!response.ok) {
      throw new Error(`Failed to fetch map data: ${response.status}`)
    }
    return response.json()
  }

  async saveMapConfig(config: GridConfig): Promise<void> {
    const response = await fetch(`${this.baseUrl}/map/config`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    })
    if (!response.ok) {
      throw new Error(`Failed to save grid config: ${response.status}`)
    }
  }
}
