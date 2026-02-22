/**
 * @seed/kit Hono 中间件
 *
 * 共享中间件，供 apps/api/edge/ 和 apps/api/bun/ 统一使用
 *
 * @example
 * ```ts
 * import {
 *   traceIdMiddleware,
 *   loggerMiddleware,
 *   transformMiddleware,
 *   errorHandlerMiddleware,
 *   authGuardMiddleware,
 *   rolesGuardMiddleware,
 *   createSessionMiddleware,
 * } from '@seed/kit/hono/middleware'
 * ```
 */
export { traceIdMiddleware } from './trace-id.middleware'
export { loggerMiddleware } from './logger.middleware'
export { transformMiddleware } from './transform.middleware'
export { errorHandlerMiddleware } from './error-handler.middleware'
export {
  authGuardMiddleware,
  rolesGuardMiddleware,
  createSessionMiddleware,
} from './auth-guard.middleware'
