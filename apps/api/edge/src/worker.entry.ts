/**
 * Cloudflare Workers 入口
 *
 * 适配 Hono 应用到 Cloudflare Workers 运行时
 * 本文件仅在 Cloudflare Workers 部署时使用
 */

import { createApp } from './app'

export type { AppType } from './app'

const { app } = createApp()

export default app
