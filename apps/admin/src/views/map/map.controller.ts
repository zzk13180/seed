import { BaseController } from '@/core/base.controller'
import type {
  MapState,
  MapEnv,
  GridConfig,
  IViewManager,
  IGridManager,
  ViewManagerOptions,
  GridEvents,
} from './map.types'
import type { Subscription } from 'rxjs'

/**
 * MapController - 继承 BaseController 获得生命周期管理
 *
 * 架构说明：
 * - 通过 Env 注入 ViewManagerFactory 和 GridManagerFactory
 * - 不直接依赖具体的 ViewManager/MapGridManager 实现
 * - 便于测试时替换为 Mock 实现
 *
 * 职责：
 * - 管理地图视图和网格的初始化/销毁
 * - 管理 RxJS 订阅
 * - 协调 IViewManager 和 IGridManager
 */
export class MapController extends BaseController<MapState, MapEnv> {
  private viewManager: IViewManager | null = null
  private gridManager: IGridManager | null = null
  private subscriptions: Subscription[] = []
  private _containerElement: HTMLElement | null = null

  /**
   * 获取视图管理器（只读）
   */
  get currentViewManager(): IViewManager | null {
    return this.viewManager
  }

  /**
   * 获取网格管理器（只读）
   */
  get currentGridManager(): IGridManager | null {
    return this.gridManager
  }

  /**
   * 获取容器元素（只读）
   */
  get containerElement(): HTMLElement | null {
    return this._containerElement
  }

  /**
   * 初始化视图管理器
   * 需要在 DOM 准备好后调用
   */
  initializeView(container: HTMLElement, options?: ViewManagerOptions): void {
    if (this.viewManager) {
      this.env.logger.warn('ViewManager already initialized')
      return
    }

    this._containerElement = container
    this.env.logger.debug('Initializing ViewManager via factory')

    try {
      // 使用工厂创建 ViewManager（通过 Env 注入）
      this.viewManager = this.env.viewManagerFactory.create(container, options)
      this.state.viewInitialized = true

      // 订阅视图状态变化
      this.subscriptions.push(
        this.viewManager.viewStateChange$.subscribe(() => {
          this.env.logger.debug('View state changed')
        }),
      )

      this.env.logger.info('ViewManager initialized')
    } catch (error) {
      this.state.errorMessage = '视图初始化失败'
      this.env.logger.error('ViewManager initialization failed', error)
      throw error
    }
  }

  /**
   * 初始化网格
   */
  initializeGrid(canvas: HTMLCanvasElement, events?: GridEvents): void {
    if (this.gridManager) {
      this.destroyGrid()
    }

    if (!this.viewManager) {
      const error = new Error('ViewManager must be initialized before grid')
      this.env.logger.error(error.message)
      throw error
    }

    this.env.logger.debug('Initializing GridManager via factory')

    try {
      // 使用工厂创建 GridManager（通过 Env 注入）
      this.gridManager = this.env.gridManagerFactory.create(
        this.state,
        canvas,
        this.viewManager,
        events,
      )
      this.gridManager.initialize()
      this.state.gridInitialized = true
      this.env.logger.info('GridManager initialized')
    } catch (error) {
      this.state.errorMessage = '网格初始化失败'
      this.env.logger.error('GridManager initialization failed', error)
      throw error
    }
  }

  /**
   * 销毁网格
   */
  destroyGrid(): void {
    if (this.gridManager) {
      this.gridManager.destroy()
      this.gridManager = null
      this.state.gridInitialized = false
      this.env.logger.debug('GridManager destroyed')
    }
  }

  /**
   * 更新网格配置
   */
  updateGridConfig(config: Partial<GridConfig>): void {
    this.state.gridConfig = { ...this.state.gridConfig, ...config }
    this.env.logger.debug('Grid config updated', config)

    // 如果 GridManager 支持 draw 方法，触发重绘
    if (this.gridManager?.draw) {
      this.gridManager.draw()
    }
  }

  /**
   * 重置视图到默认位置
   */
  resetView(): void {
    if (this.viewManager) {
      this.viewManager.resetView()
      this.env.logger.debug('View reset to default')
    }
  }

  /**
   * 设置是否允许用户交互
   */
  setInteractionEnabled(enabled: boolean): void {
    if (this.viewManager) {
      this.viewManager.setInputMovementEnabled(enabled)
      this.env.logger.debug(`Interaction ${enabled ? 'enabled' : 'disabled'}`)
    }
  }

  /**
   * 清除错误信息
   */
  clearError(): void {
    this.state.errorMessage = null
  }

  /**
   * 初始化逻辑（由 BaseController.initialize() 调用）
   */
  protected onInit(): Promise<void> {
    this.env.logger.info('MapController initializing')
    // 注意：实际的视图初始化在 initializeView 中进行
    // 因为需要 DOM 容器元素
    this.env.logger.info('MapController initialized (waiting for container)')
    return Promise.resolve()
  }

  /**
   * 销毁逻辑
   */
  protected onDispose(): Promise<void> {
    this.env.logger.info('MapController disposing')

    this.destroyGrid()
    this.destroyViewManager()
    this.state.gridInitialized = false

    this.env.logger.info('MapController disposed')
    return Promise.resolve()
  }

  /**
   * 销毁视图管理器
   */
  private destroyViewManager(): void {
    if (this.viewManager) {
      this.viewManager.destroy()
      this.viewManager = null
      this.env.logger.debug('ViewManager destroyed')
    }

    for (const sub of this.subscriptions) {
      sub.unsubscribe()
    }
    this.subscriptions = []
  }
}
