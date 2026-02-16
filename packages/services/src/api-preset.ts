/**
 * API 路由预设 — 核心路由注册工具
 *
 * 封装 api 和 server 共享的路由注册逻辑（auth + health + users），
 * 并导出路由类型供前端 Hono RPC 客户端使用。
 *
 * 解决的架构问题：
 *   admin 之前通过 `import type { AppType } from '@seed/api'` 获取 RPC 类型，
 *   违反了 `apps/* ✗ apps/*` 规则。现在 admin 从 `@seed/services` 导入类型，
 *   依赖方向变为合法的 Layer 3 → Layer 2。
 *
 * @example
 * ```ts
 * // apps/api/src/app.ts
 * import { registerCoreRoutes } from '@seed/services'
 * const routes = registerCoreRoutes(app, { auth, db: database })
 *
 * // apps/admin/src/api/client.ts
 * import type { CoreRoutesApp } from '@seed/services'
 * const api = hc<CoreRoutesApp>('')
 * ```
 */
import { createAuthRoutes } from './auth/index'
import { createHealthRoutes } from './health/index'
import { createUserRoutes } from './user/index'
import type { Hono } from 'hono'
import type { AnyDatabase } from '@seed/db'

/**
 * Auth 实例接口 — Better Auth handler 的最小契约
 */
export interface AuthHandler {
  handler: (req: Request) => Response | Promise<Response>
}

/**
 * 核心路由依赖
 */
export interface CoreRouteDeps {
  auth: AuthHandler
  db: AnyDatabase
}

/**
 * 在 Hono app 上注册核心 API 路由
 *
 * 注册的路由：
 *   - `/api/auth/**`     — Better Auth 代理
 *   - `/api/health/**`   — 健康检查（liveness + readiness）
 *   - `/api/v1/users/**` — 用户管理 CRUD
 *
 * apps/server 可在此基础上追加 server-only 路由（如 /api/updater）
 */
export function registerCoreRoutes<TApp extends Hono<any>>(app: TApp, deps: CoreRouteDeps) {
  return app
    .route('/api/auth', createAuthRoutes(deps.auth))
    .route('/api/health', createHealthRoutes(deps.db))
    .route('/api/v1/users', createUserRoutes(deps.db))
}

/**
 * 核心路由的完整 Hono 类型
 *
 * 用于 `hc<CoreRoutesApp>('')` 提供端到端类型推断。
 * 此类型编码了所有路由路径、HTTP 方法、请求/响应 schema。
 */
export type CoreRoutesApp = ReturnType<typeof registerCoreRoutes>
