/**
 * @file 网络状态监听服务
 * @description 监听网络连接状态变化，在网络断开/恢复时通知用户
 * @module core/network.service
 */

import { ElNotification } from 'element-plus'
import { createLogger } from './logger.service'

const logger = createLogger('Network')

/**
 * 网络状态
 */
export interface NetworkStatus {
  online: boolean
  type?: string
  effectiveType?: string
  downlink?: number
  rtt?: number
}

/**
 * 网络状态变化回调
 */
export type NetworkStatusCallback = (status: NetworkStatus) => void

/**
 * 网络监听服务
 */
class NetworkService {
  private isInitialized = false
  private offlineNotificationClose: (() => void) | null = null
  private wasOffline = false
  private readonly callbacks: Set<NetworkStatusCallback> = new Set()

  private readonly handleOnline = (): void => {
    logger.info('Network connection restored')

    // 关闭之前的离线通知
    if (this.offlineNotificationClose) {
      this.offlineNotificationClose()
      this.offlineNotificationClose = null
    }

    // 只有之前真的离线过才显示恢复通知
    if (this.wasOffline) {
      ElNotification({
        title: '网络已恢复',
        message: '网络连接已恢复，您可以继续操作',
        type: 'success',
        duration: 3000,
        position: 'top-right',
      })
      this.wasOffline = false
    }

    // 通知所有订阅者
    const status = this.getStatus()
    this.callbacks.forEach(callback => callback(status))
  }

  private readonly handleOffline = (): void => {
    logger.warn('Network connection lost')
    this.wasOffline = true

    // 显示离线通知（不自动关闭）
    const { close } = ElNotification({
      title: '网络已断开',
      message: '当前无法连接网络，请检查您的网络设置',
      type: 'error',
      duration: 0, // 不自动关闭
      position: 'top-right',
      showClose: true,
    })

    // 保存关闭函数以便后续关闭
    this.offlineNotificationClose = close

    // 通知所有订阅者
    const status = this.getStatus()
    this.callbacks.forEach(callback => callback(status))
  }

  /**
   * 是否在线
   */
  get isOnline(): boolean {
    return navigator.onLine
  }

  /**
   * 初始化网络监听
   */
  initialize(): void {
    if (this.isInitialized) {
      return
    }

    globalThis.addEventListener('online', this.handleOnline)
    globalThis.addEventListener('offline', this.handleOffline)

    // 初始状态检查
    if (!navigator.onLine) {
      this.handleOffline()
    }

    this.isInitialized = true
    logger.info('Network service initialized', { online: navigator.onLine })
  }

  /**
   * 销毁网络监听
   */
  dispose(): void {
    globalThis.removeEventListener('online', this.handleOnline)
    globalThis.removeEventListener('offline', this.handleOffline)
    this.callbacks.clear()
    this.isInitialized = false
    logger.info('Network service disposed')
  }

  /**
   * 获取当前网络状态
   */
  getStatus(): NetworkStatus {
    const connection = (navigator as any).connection
    return {
      online: navigator.onLine,
      type: connection?.type,
      effectiveType: connection?.effectiveType,
      downlink: connection?.downlink,
      rtt: connection?.rtt,
    }
  }

  /**
   * 订阅网络状态变化
   */
  subscribe(callback: NetworkStatusCallback): () => void {
    this.callbacks.add(callback)
    return () => {
      this.callbacks.delete(callback)
    }
  }
}

/**
 * 网络服务单例
 */
export const networkService = new NetworkService()

/**
 * 初始化网络监听
 * 在应用启动时调用
 */
export function setupNetworkListener(): () => void {
  networkService.initialize()
  return () => networkService.dispose()
}
