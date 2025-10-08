import { $http } from '@seed/http'
import type {
  ApiResponse,
  PageResult,
  UserInfo,
  UserQueryParams,
  UserCreateParams,
  UserUpdateParams,
  UserStatus,
} from './types'

/**
 * 用户管理 API
 */

/**
 * 分页查询用户
 */
export const apiGetUserPage = async (params?: UserQueryParams): Promise<PageResult<UserInfo>> => {
  const response = await $http.get<ApiResponse<PageResult<UserInfo>>>(
    '/v1/users',
    params as Record<string, unknown>,
  )
  return response.data
}

/**
 * 获取所有用户列表
 */
export const apiGetUserList = async (): Promise<UserInfo[]> => {
  const response = await $http.get<ApiResponse<UserInfo[]>>('/v1/users/list')
  return response.data
}

/**
 * 根据 ID 获取用户
 */
export const apiGetUserById = async (id: number): Promise<UserInfo> => {
  const response = await $http.get<ApiResponse<UserInfo>>(`/v1/users/${id}`)
  return response.data
}

/**
 * 创建用户
 */
export const apiCreateUser = async (params: UserCreateParams): Promise<UserInfo> => {
  const response = await $http.post<ApiResponse<UserInfo>>('/v1/users', params)
  return response.data
}

/**
 * 更新用户
 */
export const apiUpdateUser = async (id: number, params: UserUpdateParams): Promise<UserInfo> => {
  const response = await $http.put<ApiResponse<UserInfo>>(`/v1/users/${id}`, params)
  return response.data
}

/**
 * 删除用户
 */
export const apiDeleteUser = async (id: number): Promise<void> => {
  await $http.delete(`/v1/users/${id}`)
}

/**
 * 批量删除用户
 */
export const apiDeleteUserBatch = async (ids: number[]): Promise<void> => {
  await $http.post('/v1/users/batch-delete', { ids })
}

/**
 * 更新用户状态
 */
export const apiUpdateUserStatus = async (id: number, status: UserStatus): Promise<void> => {
  await $http.patch(`/v1/users/${id}/status`, { status })
}

/**
 * 重置用户密码
 */
export const apiResetUserPassword = async (id: number, password: string): Promise<void> => {
  await $http.patch(`/v1/users/${id}/password`, { password })
}
