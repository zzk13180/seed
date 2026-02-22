export interface WebSocketConfig {
  /** WebSocket 连接地址 */
  uri: string
  /** 消息处理回调函数 */
  msgHandle: (e: MessageEvent) => void
  /** 状态变更处理回调函数 */
  stateHandle: (status: Status) => void
  /** 是否启用自动重连，默认 false */
  autoReconnect?: boolean
  /** 重连延迟时间（毫秒），默认 1000 */
  reconnectDelay?: number
  /** 最大重连尝试次数，默认 3 */
  maxReconnectAttempts?: number
  /** 心跳配置 */
  heartbeat?: {
    /** 是否启用心跳检测 */
    enabled: boolean
    /** 心跳间隔时间（毫秒），默认 30000 */
    interval?: number
    /** 心跳消息内容，默认 'ping' */
    message?: string
    /** 心跳超时时间（毫秒），默认 5000 */
    timeout?: number
  }
  /** 重连失败后的回调函数 */
  onReconnectFailed?: () => void
}

export enum Status {
  /** 连接已打开 */
  OPEN = 'OPEN',
  /** 正在连接中 */
  CONNECTING = 'CONNECTING',
  /** 连接已关闭 */
  CLOSED = 'CLOSED',
  /** 连接错误 */
  ERROR = 'ERROR',
}

/**
 * WebSocket 连接器类
 * 提供 WebSocket 连接管理、自动重连、心跳检测、消息缓存等功能
 */
export class WebSocketConnector {
  /** WebSocket 实例 */
  private ws: WebSocket | null = null
  /** 当前重连尝试次数 */
  private currentReconnectAttempts: number = 0
  /** 重连定时器 */
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null
  /** 心跳定时器 */
  private heartbeatTimer: ReturnType<typeof setInterval> | null = null
  /** 心跳超时定时器 */
  private heartbeatTimeoutTimer: ReturnType<typeof setTimeout> | null = null
  /** 是否显式关闭连接（用于判断是否需要自动重连） */
  private explicitlyClosed: boolean = false
  /** 消息缓存队列，用于在连接断开时缓存消息 */
  private messageBuffer: (string | ArrayBuffer | Blob)[] = []
  /** 完整的配置对象 */
  private readonly config: Required<Omit<WebSocketConfig, 'heartbeat' | 'onReconnectFailed'>> &
    Pick<WebSocketConfig, 'heartbeat' | 'onReconnectFailed'>

  constructor(config: WebSocketConfig) {
    this.config = {
      uri: config.uri,
      msgHandle: config.msgHandle,
      stateHandle: config.stateHandle,
      autoReconnect: config.autoReconnect ?? false,
      reconnectDelay: config.reconnectDelay ?? 1000,
      maxReconnectAttempts: config.maxReconnectAttempts ?? 3,
      heartbeat: config.heartbeat,
      onReconnectFailed: config.onReconnectFailed,
    }
  }

  get status(): Status {
    if (!this.ws) return Status.CLOSED

    switch (this.ws.readyState) {
      case WebSocket.CONNECTING: {
        return Status.CONNECTING
      }
      case WebSocket.OPEN: {
        return Status.OPEN
      }
      case WebSocket.CLOSING:
      case WebSocket.CLOSED: {
        return Status.CLOSED
      }
      default: {
        return Status.ERROR
      }
    }
  }

  get isConnected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN
  }

  start(): boolean {
    // 检查浏览器是否支持 WebSocket
    if (!('WebSocket' in globalThis)) {
      console.error('当前浏览器不支持 WebSocket')
      return false
    }

    try {
      // 重置显式关闭标志
      this.explicitlyClosed = false
      // 建立连接
      this.connect()
      return true
    } catch (error) {
      console.error('WebSocket 连接失败:', error)
      this.config.stateHandle(Status.ERROR)
      return false
    }
  }

  stop(): void {
    // 设置显式关闭标志，阻止自动重连
    this.explicitlyClosed = true
    // 清理定时器
    this.clearReconnectTimer()
    this.clearHeartbeat()

    // 正常关闭 WebSocket 连接
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.close(1000, 'Client closed')
    }
  }

  send(data: string | ArrayBuffer | Blob, useBuffer: boolean = true): boolean {
    // 检查 WebSocket 实例是否存在
    if (!this.ws) {
      // 如果启用缓存，将消息添加到缓存队列
      if (useBuffer) {
        this.messageBuffer.push(data)
      }
      return false
    }

    // 检查连接状态
    if (this.ws.readyState === WebSocket.OPEN) {
      // 连接正常，先发送缓存的消息，再发送当前消息
      this.sendBufferedMessages()
      this.ws.send(data)
      return true
    } else if (useBuffer) {
      // 连接异常但启用缓存，将消息添加到缓存队列
      this.messageBuffer.push(data)
      return false
    }

    return false
  }

  updateUri(newUri: string): void {
    const wasConnected = this.isConnected
    this.config.uri = newUri

    if (wasConnected) {
      this.stop()
      setTimeout(() => this.start(), 100)
    }
  }

  destroy(): void {
    // 停止连接
    this.stop()
    // 清空消息缓存
    this.messageBuffer = []
  }

  private connect(): void {
    // 如果已有连接且状态为打开，先关闭旧连接
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.close()
    }

    // 创建新的 WebSocket 实例
    this.ws = new WebSocket(this.config.uri)
    // 通知状态变更为连接中
    this.config.stateHandle(Status.CONNECTING)

    // 绑定事件处理器
    this.ws.onopen = () => this.onOpen()
    this.ws.onclose = event => this.onClose(event)
    this.ws.onmessage = event => this.onMessage(event)
    this.ws.onerror = event => this.onError(event)
  }

  private onOpen(): void {
    console.log('WebSocket Connected', this.config.uri)
    // 重置重连计数器
    this.currentReconnectAttempts = 0
    // 通知状态变更为已打开
    this.config.stateHandle(Status.OPEN)
    // 发送缓存的消息
    this.sendBufferedMessages()
    // 启动心跳检测
    this.startHeartbeat()
  }

  private onClose(event: CloseEvent): void {
    console.log('WebSocket Closed', this.config.uri, event.code, event.reason)
    // 通知状态变更为已关闭
    this.config.stateHandle(Status.CLOSED)
    // 清理心跳相关定时器
    this.clearHeartbeat()

    // 如果不是主动关闭且启用了自动重连，尝试重连
    if (!this.explicitlyClosed && this.config.autoReconnect) {
      this.attemptReconnect()
    }
  }

  private onMessage(event: MessageEvent): void {
    // 处理心跳响应消息
    if (
      this.config.heartbeat?.enabled &&
      event.data === (this.config.heartbeat.message || 'ping')
    ) {
      this.resetHeartbeatTimeout()
      return
    }

    // 处理业务消息
    this.config.msgHandle(event)
  }

  private onError(event: Event): void {
    console.error('WebSocket Error', this.config.uri, event)
    this.config.stateHandle(Status.ERROR)
  }

  private attemptReconnect(): void {
    // 检查是否已达到最大重试次数
    if (this.currentReconnectAttempts >= this.config.maxReconnectAttempts) {
      console.error('WebSocket 重连失败，已达到最大重试次数')
      // 执行重连失败回调
      this.config.onReconnectFailed?.()
      return
    }

    // 增加重连尝试次数
    this.currentReconnectAttempts++
    console.log(
      `WebSocket 尝试重连 ${this.currentReconnectAttempts}/${this.config.maxReconnectAttempts}`,
    )

    // 设置延迟重连定时器
    this.reconnectTimer = setTimeout(() => {
      this.connect()
    }, this.config.reconnectDelay)
  }

  private clearReconnectTimer(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
  }

  private sendBufferedMessages(): void {
    if (this.messageBuffer.length > 0 && this.ws?.readyState === WebSocket.OPEN) {
      // 逐个发送缓存的消息
      for (const data of this.messageBuffer) {
        this.ws.send(data)
      }
      // 清空消息缓存
      this.messageBuffer = []
    }
  }

  private startHeartbeat(): void {
    // 检查是否启用心跳配置
    if (!this.config.heartbeat?.enabled) return

    // 获取心跳配置参数
    const interval = this.config.heartbeat.interval || 30_000
    const message = this.config.heartbeat.message || 'ping'

    // 设置心跳定时器
    this.heartbeatTimer = setInterval(() => {
      if (this.isConnected) {
        // 发送心跳消息（不使用缓存机制）
        this.send(message, false)
        // 设置心跳超时检测
        this.setHeartbeatTimeout()
      }
    }, interval)
  }

  private setHeartbeatTimeout(): void {
    if (!this.config.heartbeat?.enabled) return

    const timeout = this.config.heartbeat.timeout || 5000

    // 设置心跳超时定时器
    this.heartbeatTimeoutTimer = setTimeout(() => {
      console.warn('心跳超时，关闭连接')
      // 心跳超时，主动关闭连接
      this.ws?.close()
    }, timeout)
  }

  private resetHeartbeatTimeout(): void {
    if (this.heartbeatTimeoutTimer) {
      clearTimeout(this.heartbeatTimeoutTimer)
      this.heartbeatTimeoutTimer = null
    }
  }

  private clearHeartbeat(): void {
    // 清除心跳定时器
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer)
      this.heartbeatTimer = null
    }
    // 清除心跳超时定时器
    this.resetHeartbeatTimeout()
  }
}
