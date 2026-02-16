/**
 * @file 认证 API
 * @description 基于 Better Auth 客户端的认证操作
 * @module api/auth.api
 */

import { authClient } from './auth-client'
import type { UserVO } from './types'

/**
 * 用户登录（邮箱 + 密码）
 *
 * Better Auth 使用 cookie-session 模式，登录成功后浏览器自动携带 session cookie
 */
export const apiLogin = async (params: {
  email: string
  password: string
}): Promise<{ user: UserVO }> => {
  const { data, error } = await authClient.signIn.email({
    email: params.email,
    password: params.password,
  })

  if (error) {
    throw new Error(error.message ?? '登录失败')
  }

  return {
    user: data.user as unknown as UserVO,
  }
}

/**
 * 用户注册
 */
export const apiSignUp = async (params: {
  name: string
  email: string
  password: string
}): Promise<{ user: UserVO }> => {
  const { data, error } = await authClient.signUp.email({
    name: params.name,
    email: params.email,
    password: params.password,
  })

  if (error) {
    throw new Error(error.message ?? '注册失败')
  }

  return {
    user: data.user as unknown as UserVO,
  }
}

/**
 * 用户登出
 *
 * Better Auth 自动清除服务端 session + 客户端 cookie
 */
export const apiLogout = async (): Promise<void> => {
  await authClient.signOut()
}

/**
 * 获取当前用户信息
 *
 * 通过 session cookie 获取当前会话中的用户信息
 */
export const apiGetCurrentUser = async (): Promise<UserVO | null> => {
  const { data } = await authClient.getSession()

  if (!data) {
    return null
  }

  return data.user as unknown as UserVO
}

/**
 * 检查是否已认证
 *
 * 检查当前浏览器是否有有效的 session
 */
export const apiCheckAuth = async (): Promise<boolean> => {
  const { data } = await authClient.getSession()
  return !!data?.session
}
