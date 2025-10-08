import { useStorage, type RemovableRef } from '@vueuse/core'

const ACCESS_TOKEN_KEY = 'access_token'
const REFRESH_TOKEN_KEY = 'refresh_token'
const TOKEN_EXPIRES_KEY = 'token_expires_at'

/**
 * 令牌管理工具类
 * 支持 access token 和 refresh token 的管理
 */
export class AccessTokenUtil {
  /**
   * 获取 access token
   */
  static get token(): string | null {
    return AccessTokenUtil.accessTokenStorage.value
  }

  /**
   * 获取 refresh token
   */
  static get refreshToken(): string | null {
    return AccessTokenUtil.refreshTokenStorage.value
  }

  /**
   * 获取过期时间戳
   */
  static get expiresAt(): number | null {
    return AccessTokenUtil.expiresAtStorage.value
  }

  /**
   * 检查 token 是否已过期
   */
  static get isExpired(): boolean {
    const expiresAt = AccessTokenUtil.expiresAt
    if (!expiresAt) {
      return true
    }
    // 提前 30 秒判定为过期，预留刷新时间
    return Date.now() >= expiresAt - 30_000
  }

  /**
   * 检查是否有有效的 token
   */
  static get isAuthenticated(): boolean {
    return !!AccessTokenUtil.token && !AccessTokenUtil.isExpired
  }

  /**
   * 设置令牌
   * @param accessToken access token
   * @param refreshToken refresh token（可选）
   * @param expiresIn 过期时间（秒）
   */
  static setTokens(accessToken: string, refreshToken?: string, expiresIn?: number): void {
    AccessTokenUtil.accessTokenStorage.value = accessToken

    if (refreshToken) {
      AccessTokenUtil.refreshTokenStorage.value = refreshToken
    }

    if (expiresIn) {
      AccessTokenUtil.expiresAtStorage.value = Date.now() + expiresIn * 1000
    }
  }

  /**
   * 设置 access token（兼容旧 API）
   */
  static setToken(accessToken: string): void {
    AccessTokenUtil.accessTokenStorage.value = accessToken
  }

  /**
   * 清除所有令牌
   */
  static removeToken(): void {
    AccessTokenUtil.accessTokenStorage.value = null
    AccessTokenUtil.refreshTokenStorage.value = null
    AccessTokenUtil.expiresAtStorage.value = null
  }

  /**
   * 清除所有令牌（别名）
   */
  static clear(): void {
    AccessTokenUtil.removeToken()
  }

  private static readonly accessTokenStorage: RemovableRef<null | string> = useStorage<
    null | string
  >(ACCESS_TOKEN_KEY, null)

  private static readonly refreshTokenStorage: RemovableRef<null | string> = useStorage<
    null | string
  >(REFRESH_TOKEN_KEY, null)

  private static readonly expiresAtStorage: RemovableRef<null | number> = useStorage<null | number>(
    TOKEN_EXPIRES_KEY,
    null,
  )

  private constructor() {}
}
