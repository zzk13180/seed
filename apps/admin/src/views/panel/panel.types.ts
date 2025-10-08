/**
 * Panel 模块的类型定义
 * 纯 TypeScript 类型，不依赖任何框架
 */

import type { Logger } from '@/core/logger.service'
import type { ErrorHandler } from '@/core/error.service'

/**
 * 机器人信息
 */
export interface RobotInfo {
  robotId?: string
  name?: string
  status?: 'online' | 'offline' | 'error'
  batteryLevel?: number
  lastUpdated?: string
}

/**
 * Panel 状态接口
 */
export interface PanelState {
  /** 机器人信息 */
  robotInfo: RobotInfo
  /** 是否正在加载 */
  loading: boolean
  /** 错误信息 */
  errorMessage: string | null
  /** WebSocket 连接状态 */
  wsConnected: boolean
}

/**
 * Panel API 服务接口
 */
export interface PanelApiService {
  fetchRobotInfo(): Promise<RobotInfo>
  updateRobotInfo(info: Partial<RobotInfo>): Promise<RobotInfo>
}

/**
 * WebSocket 服务接口
 */
export interface PanelWsService {
  connect(url: string): void
  disconnect(): void
  isConnected(): boolean
  onMessage(callback: (data: unknown) => void): void
  send(data: unknown): void
}

/**
 * Panel 模块环境依赖
 */
export interface PanelEnv {
  logger: Logger
  apiService: PanelApiService
  wsService?: PanelWsService
  errorHandler: ErrorHandler
}
