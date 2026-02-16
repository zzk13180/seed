/**
 * Re-export from shared package + admin-specific factory
 * @see packages/core/src/frontend/logger.service.ts
 */
import { createLogger as _createLogger } from '@seed/kit/frontend'
import type { Logger, LogLevel } from '@seed/kit/frontend'

export { ConsoleLogger, SilentLogger } from '@seed/kit/frontend'
export type { Logger, LogLevel } from '@seed/kit/frontend'

/**
 * Admin 专用日志工厂 — 使用 import.meta.env.PROD 检测环境
 */
export function createLogger(prefix: string, minLevel?: LogLevel): Logger {
  return _createLogger(prefix, minLevel, import.meta.env.PROD)
}
