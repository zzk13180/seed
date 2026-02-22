/**
 * @seed/kit/frontend — 前端 Controller 基础设施
 *
 * 提供 BaseController 和 Logger，所有前端 app 共享:
 * - apps/spa/admin/ (Vue + Element Plus)
 * - apps/spa/console/ (Vue + Ionic)
 * - 未来新增的前端 app
 */

// Controller 基类
export { BaseController } from './base.controller'

// Logger 服务
export { ConsoleLogger, SilentLogger, createLogger } from './logger.service'
export type { Logger, LogLevel } from './logger.service'
