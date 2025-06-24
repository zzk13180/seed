export interface SSEConfig {
  /** SSE 连接地址 */
  url: string
  /** 消息处理回调函数 */
  onMessage: (event: MessageEvent) => void
  /** 状态变更处理回调函数 */
  onStateChange: (status: SSEStatus) => void
  /** 请求头配置 */
  headers?: Record<string, string>
  /** 是否启用自动重连，默认 false */
  autoReconnect?: boolean
  /** 重连延迟时间（毫秒），默认 1000 */
  reconnectDelay?: number
  /** 最大重连尝试次数，默认 3 */
  maxReconnectAttempts?: number
  /** 连接超时时间（毫秒），默认 10000 */
  timeout?: number
  /** 重连失败后的回调函数 */
  onReconnectFailed?: () => void
  /** 错误处理回调函数 */
  onError?: (error: Error) => void
}

export enum SSEStatus {
  /** 连接已打开 */
  OPEN = 'OPEN',
  /** 正在连接中 */
  CONNECTING = 'CONNECTING',
  /** 连接已关闭 */
  CLOSED = 'CLOSED',
  /** 连接错误 */
  ERROR = 'ERROR',
}

export class SSEConnector {
  /** EventSource 实例 */
  private eventSource: EventSource | null = null
  /** 记录上次的 lastEventId，重连时携带 */
  private lastEventId: string | null = null
  /** 当前重连尝试次数 */
  private currentReconnectAttempts: number = 0
  /** 重连定时器 */
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null
  /** 连接超时定时器 */
  private timeoutTimer: ReturnType<typeof setTimeout> | null = null
  /** 是否显式关闭连接（用于判断是否需要自动重连） */
  private explicitlyClosed: boolean = false
  /** 完整的配置对象 */
  private readonly config: Required<Omit<SSEConfig, 'headers' | 'onReconnectFailed' | 'onError'>> &
    Pick<SSEConfig, 'headers' | 'onReconnectFailed' | 'onError'>

  constructor(config: SSEConfig) {
    this.config = {
      url: config.url,
      onMessage: config.onMessage,
      onStateChange: config.onStateChange,
      headers: config.headers,
      autoReconnect: config.autoReconnect ?? false,
      reconnectDelay: config.reconnectDelay ?? 1000,
      maxReconnectAttempts: config.maxReconnectAttempts ?? 3,
      timeout: config.timeout ?? 10_000,
      onReconnectFailed: config.onReconnectFailed,
      onError: config.onError,
    }
  }

  get status(): SSEStatus {
    if (!this.eventSource) return SSEStatus.CLOSED

    switch (this.eventSource.readyState) {
      case EventSource.CONNECTING: {
        return SSEStatus.CONNECTING
      }
      case EventSource.OPEN: {
        return SSEStatus.OPEN
      }
      case EventSource.CLOSED: {
        return SSEStatus.CLOSED
      }
      default: {
        return SSEStatus.ERROR
      }
    }
  }

  get isConnected(): boolean {
    return this.eventSource?.readyState === EventSource.OPEN
  }

  start(): boolean {
    // 检查浏览器是否支持 EventSource
    if (!('EventSource' in globalThis)) {
      const error = new Error('当前浏览器不支持 Server-Sent Events')
      console.error(error.message)
      this.config.onError?.(error)
      return false
    }

    try {
      // 重置显式关闭标志
      this.explicitlyClosed = false
      // 建立连接
      this.connect()
      return true
    } catch (error) {
      console.error('SSE 连接失败:', error)
      this.config.onStateChange(SSEStatus.ERROR)
      this.config.onError?.(error as Error)
      return false
    }
  }

  stop(): void {
    // 设置显式关闭标志，阻止自动重连
    this.explicitlyClosed = true
    // 清理定时器
    this.clearReconnectTimer()
    this.clearTimeoutTimer()

    // 关闭 EventSource 连接
    if (this.eventSource) {
      this.eventSource.onopen = null
      this.eventSource.onmessage = null
      this.eventSource.onerror = null
      this.eventSource.close()
      this.eventSource = null
    }
    // 通知状态已关闭
    this.config.onStateChange(SSEStatus.CLOSED)
  }

  updateUrl(newUrl: string): void {
    const wasConnected = this.isConnected
    this.config.url = newUrl

    if (wasConnected) {
      this.stop()
      setTimeout(() => this.start(), 100)
    }
  }

  destroy(): void {
    this.stop()
  }

  private connect(): void {
    // 每次重连前，先清理上次的定时器和实例
    this.clearReconnectTimer()
    this.clearTimeoutTimer()
    if (this.eventSource) {
      this.eventSource.onopen = null
      this.eventSource.onmessage = null
      this.eventSource.onerror = null
      this.eventSource.close()
      this.eventSource = null
    }

    const url = this.buildUrlWithHeaders()
    this.eventSource = new EventSource(url)
    this.config.onStateChange(SSEStatus.CONNECTING)
    this.setConnectionTimeout()

    this.eventSource.onopen = () => this.onOpen()
    this.eventSource.onmessage = e => this.onMessage(e)
    this.eventSource.onerror = e => this.onError(e)
  }

  /** 构建带 headers 和 lastEventId 的 URL */
  private buildUrlWithHeaders(): string {
    const urlObj = new URL(this.config.url, globalThis.location.origin)
    if (this.config.headers) {
      for (const [key, val] of Object.entries(this.config.headers)) {
        urlObj.searchParams.set(`header_${key}`, val)
      }
    }
    if (this.lastEventId) {
      urlObj.searchParams.set('lastEventId', this.lastEventId)
    }
    return urlObj.toString()
  }

  private setConnectionTimeout(): void {
    this.timeoutTimer = setTimeout(() => {
      if (this.eventSource?.readyState === EventSource.CONNECTING) {
        console.warn('SSE 连接超时')
        this.eventSource.close()
        const error = new Error('连接超时')
        this.config.onError?.(error)
        this.config.onStateChange(SSEStatus.ERROR)
      }
    }, this.config.timeout)
  }

  private clearTimeoutTimer(): void {
    if (this.timeoutTimer) {
      clearTimeout(this.timeoutTimer)
      this.timeoutTimer = null
    }
  }

  private onOpen(): void {
    // 连接成功，清理所有重连/超时定时器
    this.clearTimeoutTimer()
    this.clearReconnectTimer()
    this.currentReconnectAttempts = 0
    this.config.onStateChange(SSEStatus.OPEN)
  }

  private onMessage(event: MessageEvent): void {
    // 更新 lastEventId，方便断线重连时续传
    this.lastEventId = event.lastEventId || this.lastEventId
    this.config.onMessage(event)
  }

  private onError(event: Event): void {
    this.clearTimeoutTimer()
    // 区分是连接关闭还是错误
    const ready = this.eventSource?.readyState
    const status = ready === EventSource.CLOSED ? SSEStatus.CLOSED : SSEStatus.ERROR
    this.config.onError?.(new Error('SSE 连接错误'))
    this.config.onStateChange(status)

    // 自动重连逻辑
    if (!this.explicitlyClosed && this.config.autoReconnect) {
      this.attemptReconnect()
    }
  }

  private attemptReconnect(): void {
    // 检查是否已达到最大重试次数
    if (this.currentReconnectAttempts >= this.config.maxReconnectAttempts) {
      console.error('SSE 重连失败，已达到最大重试次数')
      // 执行重连失败回调
      this.config.onReconnectFailed?.()
      return
    }

    // 增加重连尝试次数
    this.currentReconnectAttempts++
    console.log(`SSE 尝试重连 ${this.currentReconnectAttempts}/${this.config.maxReconnectAttempts}`)

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
}
