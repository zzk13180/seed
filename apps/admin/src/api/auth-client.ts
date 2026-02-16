/**
 * @file Better Auth 前端客户端
 * @description 会话管理、登录/登出/注册，基于 Better Auth 的 cookie-session 模式
 * @module api/auth-client
 */

import { createAuthClient } from 'better-auth/client'

/**
 * Better Auth 客户端实例
 *
 * - Web（admin）使用 cookie 模式，浏览器自动携带 session cookie
 * - 认证端点: /api/auth/**
 */
export const authClient = createAuthClient({
  baseURL: `${globalThis.location.origin}/api/auth`,
})

// 导出常用方法的快捷引用
export const { signIn, signUp, signOut, getSession, useSession } = authClient
