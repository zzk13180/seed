import { createHonoApp } from '@seed/kit/hono/app-factory'
import { registerCoreRoutes } from '@seed/services'
import { auth } from './config/auth.config'
import { isDev } from './config/env.config'
import { db } from './database/database.provider'

/**
 * 创建 Hono 应用实例
 *
 * api/edge 部署到 Cloudflare Workers，仅注册无状态 CRUD 模块
 * 文件系统/WebSocket/Cron 等重型任务由 api/bun 处理
 */
export function createApp() {
  const app = createHonoApp({ auth, isDev: isDev() })
  const routes = registerCoreRoutes(app, { auth, db: db() })

  return { app, routes }
}

export type AppType = ReturnType<typeof createApp>['routes']
