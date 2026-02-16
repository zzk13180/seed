/**
 * 日志级别
 */
export type LogLevel = 'debug' | 'info' | 'warn' | 'error'

/**
 * Logger 接口 — 抽象日志服务
 * Controller 通过接口依赖，不直接依赖具体实现
 */
export interface Logger {
  debug(message: string, ...args: unknown[]): void
  info(message: string, ...args: unknown[]): void
  warn(message: string, ...args: unknown[]): void
  error(message: string, ...args: unknown[]): void
}

/**
 * 控制台日志实现
 */
export class ConsoleLogger implements Logger {
  private readonly prefix: string
  private readonly minLevel: LogLevel

  private readonly levelOrder: Record<LogLevel, number> = {
    debug: 0,
    info: 1,
    warn: 2,
    error: 3,
  }

  constructor(prefix: string = '', minLevel: LogLevel = 'debug') {
    this.prefix = prefix ? `[${prefix}]` : ''
    this.minLevel = minLevel
  }

  debug(message: string, ...args: unknown[]): void {
    if (this.shouldLog('debug')) {
      console.debug(this.formatMessage(message), ...args)
    }
  }

  info(message: string, ...args: unknown[]): void {
    if (this.shouldLog('info')) {
      console.info(this.formatMessage(message), ...args)
    }
  }

  warn(message: string, ...args: unknown[]): void {
    if (this.shouldLog('warn')) {
      console.warn(this.formatMessage(message), ...args)
    }
  }

  error(message: string, ...args: unknown[]): void {
    if (this.shouldLog('error')) {
      console.error(this.formatMessage(message), ...args)
    }
  }

  private shouldLog(level: LogLevel): boolean {
    return this.levelOrder[level] >= this.levelOrder[this.minLevel]
  }

  private formatMessage(message: string): string {
    const timestamp = new Date().toISOString()
    return `${timestamp} ${this.prefix} ${message}`
  }
}

/**
 * 静默日志实现（用于测试或生产环境）
 */
export class SilentLogger implements Logger {
  debug(): void {}
  info(): void {}
  warn(): void {}
  error(): void {}
}

/**
 * 创建日志实例的工厂函数
 *
 * 生产环境自动使用 warn 级别，开发环境使用 debug 级别（或自定义）
 *
 * @param prefix - 日志前缀（通常为模块名）
 * @param minLevel - 最低日志级别
 * @param isProd - 是否为生产环境（默认 false）
 */
export function createLogger(prefix: string, minLevel?: LogLevel, isProd = false): Logger {
  if (isProd) {
    return new ConsoleLogger(prefix, 'warn')
  }
  return new ConsoleLogger(prefix, minLevel)
}
