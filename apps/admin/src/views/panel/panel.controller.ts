import { BaseController } from '@/core/base.controller'
import type { PanelState, PanelEnv, RobotInfo } from './panel.types'

/**
 * PanelController - 继承 BaseController 获得生命周期管理
 *
 * 职责：
 * - 处理机器人面板相关业务逻辑
 * - 管理 WebSocket 连接
 * - 管理副作用（API 调用、定时器等）
 *
 * 如果未来迁移到 React/Svelte，只需要重写 Store 适配层，
 * 这个 Controller 可以完全复用。
 */
export class PanelController extends BaseController<PanelState, PanelEnv> {
  private refreshTimer: ReturnType<typeof setInterval> | null = null

  /**
   * 获取当前机器人 ID
   */
  get robotId(): string {
    return this.state.robotInfo.robotId || ''
  }

  /**
   * 获取机器人状态
   */
  get robotStatus(): string {
    return this.state.robotInfo.status || 'offline'
  }

  /**
   * 加载机器人信息
   */
  async loadRobotInfo(): Promise<void> {
    this.state.loading = true
    this.state.errorMessage = null

    try {
      const info = await this.env.apiService.fetchRobotInfo()
      this.state.robotInfo = info
      this.env.logger.debug('Robot info loaded', info)
    } catch (error) {
      // 使用统一的错误处理服务获取用户友好的错误消息
      const message = this.env.errorHandler.getUserMessage(error)
      this.state.errorMessage = message
      this.env.logger.error('Failed to load robot info', error)
    } finally {
      this.state.loading = false
    }
  }

  /**
   * 更新机器人信息
   */
  async updateRobotInfo(info: Partial<RobotInfo>): Promise<void> {
    this.state.loading = true

    try {
      const updated = await this.env.apiService.updateRobotInfo(info)
      this.state.robotInfo = updated
      this.env.logger.info('Robot info updated', updated)
    } catch (error) {
      // 使用统一的错误处理服务获取用户友好的错误消息
      const message = this.env.errorHandler.getUserMessage(error)
      this.state.errorMessage = message
      this.env.logger.error('Failed to update robot info', error)
      throw error
    } finally {
      this.state.loading = false
    }
  }

  /**
   * 清除错误信息
   */
  clearError(): void {
    this.state.errorMessage = null
  }

  /**
   * 初始化逻辑
   */
  protected async onInit(): Promise<void> {
    this.env.logger.info('PanelController initializing')

    // 加载机器人信息
    await this.loadRobotInfo()

    // 启动定时刷新（每30秒）
    this.startAutoRefresh(30_000)

    // 连接 WebSocket（如果有服务）
    if (this.env.wsService) {
      this.connectWebSocket()
    }

    this.env.logger.info('PanelController initialized')
  }

  /**
   * 销毁逻辑
   */
  protected onDispose(): Promise<void> {
    this.env.logger.info('PanelController disposing')

    // 停止定时刷新
    this.stopAutoRefresh()

    // 断开 WebSocket
    if (this.env.wsService) {
      this.env.wsService.disconnect()
      this.state.wsConnected = false
    }

    this.env.logger.info('PanelController disposed')
    return Promise.resolve()
  }

  /**
   * 启动自动刷新
   */
  private startAutoRefresh(interval: number): void {
    this.stopAutoRefresh()
    this.refreshTimer = setInterval(() => {
      this.loadRobotInfo().catch(error => {
        this.env.logger.warn('Auto refresh failed', error)
      })
    }, interval)
    this.env.logger.debug('Auto refresh started', { interval })
  }

  /**
   * 停止自动刷新
   */
  private stopAutoRefresh(): void {
    if (this.refreshTimer) {
      clearInterval(this.refreshTimer)
      this.refreshTimer = null
      this.env.logger.debug('Auto refresh stopped')
    }
  }

  /**
   * 连接 WebSocket
   */
  private connectWebSocket(): void {
    if (!this.env.wsService) return

    this.env.wsService.onMessage(data => {
      this.handleWsMessage(data)
    })

    // 实际连接逻辑可以根据需要添加
    this.state.wsConnected = true
    this.env.logger.info('WebSocket connected')
  }

  /**
   * 处理 WebSocket 消息
   */
  private handleWsMessage(data: unknown): void {
    this.env.logger.debug('WebSocket message received', data)
    // 根据消息类型处理
    if (typeof data === 'object' && data !== null && 'robotInfo' in data) {
      this.state.robotInfo = {
        ...this.state.robotInfo,
        ...(data as { robotInfo: RobotInfo }).robotInfo,
      }
    }
  }
}
