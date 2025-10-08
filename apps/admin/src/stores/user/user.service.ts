import { $http } from '@seed/http'
import { AccessTokenUtil } from '@/utils/token.util'
import type { AuthService, StorageService, LoginParams, LoginResult, UserInfo } from './user.types'

/**
 * HTTP 认证服务实现
 */
export class HttpAuthService implements AuthService {
  async login(params: LoginParams): Promise<LoginResult> {
    const response = await $http.post<{ data: LoginResult }>('/auth/login', params)
    const { accessToken, refreshToken, expiresIn } = response.data

    // 保存令牌
    AccessTokenUtil.setTokens(accessToken, refreshToken, expiresIn)

    // 设置 HTTP 客户端 Authorization 头
    $http.setAuthorization(accessToken)

    return response.data
  }

  async logout(): Promise<void> {
    try {
      await $http.post('/auth/logout')
    } finally {
      AccessTokenUtil.clear()
      $http.clearAuthorization()
    }
  }

  async getCurrentUser(): Promise<UserInfo> {
    const response = await $http.get<{ data: UserInfo }>('/auth/me')
    return response.data
  }

  async refreshToken(): Promise<LoginResult> {
    const refreshToken = AccessTokenUtil.refreshToken
    if (!refreshToken) {
      throw new Error('No refresh token available')
    }

    const response = await $http.post<{ data: LoginResult }>('/auth/refresh', { refreshToken })
    const { accessToken, refreshToken: newRefreshToken, expiresIn } = response.data

    AccessTokenUtil.setTokens(accessToken, newRefreshToken, expiresIn)
    $http.setAuthorization(accessToken)

    return response.data
  }

  isAuthenticated(): boolean {
    return AccessTokenUtil.isAuthenticated
  }

  initAuth(): boolean {
    const token = AccessTokenUtil.token
    if (token && !AccessTokenUtil.isExpired) {
      $http.setAuthorization(token)
      return true
    }
    return false
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
