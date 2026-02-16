/**
 * @file Hono RPC 前端客户端
 * @description 基于 Hono Client (hc) 的端到端类型安全 API 客户端
 * @module api/client
 *
 * 使用方式:
 *   import { api } from '@/api/client'
 *   const res = await api.api.v1.users.$get({ query: { page: 1, pageSize: 10 } })
 *   const data = await res.json()
 */

import { hc } from 'hono/client'
import type { CoreRoutesApp } from '@seed/services'

/**
 * Hono RPC 客户端
 *
 * - 使用空字符串作为 baseURL，请求路径为相对路径（例如 /api/v1/users）
 * - 开发环境下通过 Vite proxy 转发到后端
 * - 生产环境下通过 Nginx/CDN 反代转发
 * - credentials: 'include' 确保 Better Auth session cookie 被携带
 */
export const api = hc<CoreRoutesApp>('', {
  fetch: (input: RequestInfo | URL, init?: RequestInit) =>
    fetch(input, {
      ...init,
      credentials: 'include',
    }),
})
