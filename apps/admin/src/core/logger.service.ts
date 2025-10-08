/**
 * @file 日志服务
 * @description 提供日志抽象层，支持 Controller 通过接口依赖注入
 * @module core/logger.service
 *
 * @exports Logger - 日志接口，Controller 依赖此接口
 * @exports ConsoleLogger - 控制台日志实现
 * @exports SilentLogger - 静默日志实现（测试用）
 * @exports createLogger - 日志工厂函数
 * @exports LogLevel - 日志级别类型
 */

/**
 * 日志级别
 * @typedef {'debug' | 'info' | 'warn' | 'error'} LogLevel
 */
export type LogLevel = 'debug' | 'info' | 'warn' | 'error'

/**
 * Logger 接口 - 抽象日志服务
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
 */
export function createLogger(prefix: string, minLevel?: LogLevel): Logger {
  if (import.meta.env.PROD) {
    return new ConsoleLogger(prefix, 'warn')
  }
  return new ConsoleLogger(prefix, minLevel)
}
