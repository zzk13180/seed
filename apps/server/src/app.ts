import { createHonoApp } from '@seed/kit/hono/app-factory'
import { registerCoreRoutes, createTauriUpdaterRoutes } from '@seed/services'
import { auth } from './config/auth.config'
import { isDev } from './config/env.config'
import { createDatabase } from './database/database.provider'

/**
 * 创建 Hono 应用实例
 *
 * server 部署到 Docker (Bun)，使用 node-postgres TCP 长连接
 * 注册所有模块，包括需要文件系统的 Tauri Updater
 * 可扩展: WebSocket、Cron 等重型任务
 */
export function createApp() {
  const app = createHonoApp({ auth, isDev: isDev() })
  const database = createDatabase()

  // 注册核心路由（与 api 共享）
  const coreRoutes = registerCoreRoutes(app, { auth, db: database })

  // 注册 server-only 路由
  const routes = coreRoutes.route('/api/updater', createTauriUpdaterRoutes())

  // server 可在此扩展:
  // routes.route('/api/v1/ws', createWebSocketRoutes(...))
  // routes.route('/api/v1/cron', createCronRoutes(...))

  return { app, routes }
}

export type AppType = ReturnType<typeof createApp>['routes']
