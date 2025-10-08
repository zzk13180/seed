/**
 * @file 控制器基类
 * @description 提供统一的生命周期管理，所有业务 Controller 应继承此类
 * @module core/base.controller
 *
 * @exports BaseController - 抽象基类，提供 initialize/dispose 生命周期
 *
 * @example
 * ```typescript
 * class PanelController extends BaseController<PanelState, PanelEnv> {
 *   protected async onInit() { ... }
 *   protected async onDispose() { ... }
 * }
 * ```
 */

/**
 * BaseController - 抽象基类
 *
 * 提供通用的生命周期管理：
 * - 异步初始化，支持竞态保护
 * - 并发调用时返回同一个 Promise，避免重复初始化
 * - 初始化失败后可重试
 *
 * 子类只需实现 onInit() 和 onDispose() 方法
 */
export abstract class BaseController<TState, TEnv> {
  protected initialized = false
  private initPromise: Promise<void> | null = null

  constructor(
    protected readonly state: TState,
    protected readonly env: TEnv,
  ) {}

  /**
   * 检查控制器是否已初始化
   */
  get isInitialized(): boolean {
    return this.initialized
  }

  /**
   * 异步初始化，支持竞态保护
   * - 并发调用时返回同一个 Promise，避免重复初始化
   * - 初始化失败后可重试
   */
  async initialize(): Promise<void> {
    // 防止并发初始化：如果已有初始化进程，直接返回
    if (this.initPromise) return this.initPromise
    // 已初始化完成，直接返回
    if (this.initialized) return

    this.initPromise = this.onInit()
      .then(() => {
        this.initialized = true
      })
      .catch(error => {
        // 初始化失败时清空 Promise，允许重试
        this.initPromise = null
        throw error
      })

    return this.initPromise
  }

  /**
   * 销毁控制器，清理资源
   */
  async dispose(): Promise<void> {
    if (!this.initialized) return

    this.initialized = false
    this.initPromise = null
    await this.onDispose()
  }

  /**
   * 子类实现：初始化逻辑
   */
  protected abstract onInit(): Promise<void>

  /**
   * 子类实现：销毁逻辑
   */
  protected abstract onDispose(): Promise<void>
}
