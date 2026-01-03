import { $http } from '@seed/http'
import { AccessTokenUtil } from '@/utils/token.util'
import type { ApiResponse, ILoginDto, ILoginVo, IUserVo } from './types'

/**
 * 认证相关 API
 */

/**
 * 用户登录
 */
export const apiLogin = async (params: ILoginDto): Promise<ILoginVo> => {
  const response = await $http.post<ApiResponse<ILoginVo>>('/auth/login', params)
  const { accessToken, refreshToken, expiresIn } = response.data

  // 保存令牌
  AccessTokenUtil.setTokens(accessToken, refreshToken, expiresIn)

  // 设置 HTTP 客户端 Authorization 头
  $http.setAuthorization(accessToken)

  return response.data
}

/**
 * 刷新令牌
 */
export const apiRefreshToken = async (): Promise<ILoginVo> => {
  const refreshToken = AccessTokenUtil.refreshToken
  if (!refreshToken) {
    throw new Error('No refresh token available')
  }

  const response = await $http.post<ApiResponse<ILoginVo>>('/auth/refresh', { refreshToken })
  const { accessToken, refreshToken: newRefreshToken, expiresIn } = response.data

  // 更新令牌
  AccessTokenUtil.setTokens(accessToken, newRefreshToken, expiresIn)
  $http.setAuthorization(accessToken)

  return response.data
}

/**
 * 用户登出
 */
export const apiLogout = async (): Promise<void> => {
  try {
    await $http.post('/auth/logout')
  } finally {
    // 清除令牌
    AccessTokenUtil.clear()
    $http.clearAuthorization()
  }
}

/**
 * 获取当前用户信息
 */
export const apiGetCurrentUser = async (): Promise<IUserVo> => {
  const response = await $http.get<ApiResponse<IUserVo>>('/auth/me')
  return response.data
}

/**
 * 初始化认证
 * 在应用启动时调用，恢复已保存的令牌
 */
export const initAuth = (): boolean => {
  const token = AccessTokenUtil.token
  if (token && !AccessTokenUtil.isExpired) {
    $http.setAuthorization(token)
    return true
  }
  return false
}
