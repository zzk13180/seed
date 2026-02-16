import { authClient } from '@/api/auth-client'
import type { UserVO } from '@seed/contracts'
import type { AuthService, LoginParams, StorageService } from './user.types'

/**
 * Better Auth 认证服务实现
 *
 * 使用 cookie-session 模式:
 * - 登录后浏览器自动携带 session cookie
 * - 无需手动管理 token
 */
export class BetterAuthService implements AuthService {
  async login(params: LoginParams): Promise<UserVO> {
    const { data, error } = await authClient.signIn.email({
      email: params.email,
      password: params.password,
    })

    if (error) {
      throw new Error(error.message ?? '登录失败')
    }

    return data.user as unknown as UserVO
  }

  async logout(): Promise<void> {
    await authClient.signOut()
  }

  async getCurrentUser(): Promise<UserVO | null> {
    const { data } = await authClient.getSession()
    if (!data?.user) return null
    return data.user as unknown as UserVO
  }

  async isAuthenticated(): Promise<boolean> {
    const { data } = await authClient.getSession()
    return !!data?.session
  }
}

/**
 * LocalStorage 存储服务实现
 */
export class LocalStorageService implements StorageService {
  get(key: string): string | null {
    return localStorage.getItem(key)
  }

  set(key: string, value: string): void {
    localStorage.setItem(key, value)
  }

  remove(key: string): void {
    localStorage.removeItem(key)
  }
}
