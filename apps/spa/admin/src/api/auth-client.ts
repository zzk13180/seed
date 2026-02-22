/**
 * @file Better Auth 前端客户端
 * @description 会话管理、登录/登出/注册，基于 Better Auth 的 Bearer Token 模式
 * @module api/auth-client
 */

import { createAuthClient } from 'better-auth/client'

const TOKEN_KEY = 'seed_auth_token'

/**
 * 获取存储的认证 Token
 */
function getStoredToken(): string | null {
  try {
    return localStorage.getItem(TOKEN_KEY)
  } catch {
    return null
  }
}

/**
 * 存储认证 Token
 */
export function storeToken(token: string): void {
  localStorage.setItem(TOKEN_KEY, token)
}

/**
 * 清除认证 Token
 */
export function clearToken(): void {
  localStorage.removeItem(TOKEN_KEY)
}

/**
 * Better Auth 客户端实例
 *
 * - 统一使用 Bearer Token 模式，全端一致（Web / Native / Mobile）
 * - 认证端点: /api/auth/**
 * - Token 存储在 localStorage（Web）或 Secure Storage（Native）
 */
export const authClient = createAuthClient({
  baseURL: `${globalThis.location.origin}/api/auth`,
  fetchOptions: {
    onRequest(ctx) {
      const token = getStoredToken()
      if (token) {
        ctx.headers.set('Authorization', `Bearer ${token}`)
      }
    },
  },
})

// 导出常用方法的快捷引用
export const { signIn, signUp, signOut, getSession, useSession } = authClient
