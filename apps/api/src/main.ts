/**
 * Bun 运行时入口
 *
 * 开发环境和 Docker 部署时使用此入口
 * Cloudflare Workers 部署使用 worker.ts
 */

import { env } from './config/env.config'
import { createApp } from './app'

const { app } = createApp()
const port = env().PORT

console.log(`🚀 Seed API server running on http://localhost:${port}`)
console.log(`📚 Auth endpoints: http://localhost:${port}/api/auth`)
console.log(`❤️  Health check: http://localhost:${port}/api/health`)

export default {
  port,
  fetch: app.fetch,
}
