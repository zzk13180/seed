/**
 * @file 用户管理 API
 * @description 基于 Hono RPC 的端到端类型安全用户接口
 * @module api/user.api
 */

import { api } from './client'
import type {
  PageResult,
  UserVO,
  UserQueryDto,
  UserCreateDto,
  UserUpdateDto,
  UserStatus,
} from './types'

/**
 * 分页查询用户
 */
export const apiGetUserPage = async (params?: UserQueryDto): Promise<PageResult<UserVO>> => {
  const res = await api.api.v1.users.$get({
    query: params as unknown as Record<string, string>,
  })
  return res.json() as Promise<PageResult<UserVO>>
}

/**
 * 获取所有用户列表
 */
export const apiGetUserList = async (): Promise<UserVO[]> => {
  const res = await api.api.v1.users.list.$get()
  return res.json() as Promise<UserVO[]>
}

/**
 * 根据 ID 获取用户
 */
export const apiGetUserById = async (id: string): Promise<UserVO> => {
  const res = await api.api.v1.users[':id'].$get({
    param: { id },
  })
  return res.json() as Promise<UserVO>
}

/**
 * 创建用户（通过 Better Auth signUp）
 */
export const apiCreateUser = async (params: UserCreateDto): Promise<UserVO> => {
  // 用户创建通过 Better Auth signUp.email 实现
  // 前端管理后台创建用户等同于代为注册
  const { authClient } = await import('./auth-client')
  const { data, error } = await authClient.signUp.email({
    name: params.username,
    email: params.email || `${params.username}@placeholder.local`,
    password: params.password,
  })
  if (error) throw new Error(error.message ?? '创建用户失败')
  return data.user as unknown as UserVO
}

/**
 * 更新用户
 */
export const apiUpdateUser = async (id: string, params: UserUpdateDto): Promise<UserVO> => {
  const res = await api.api.v1.users[':id'].$put({
    param: { id },
    json: params,
  })
  return res.json() as Promise<UserVO>
}

/**
 * 删除用户
 */
export const apiDeleteUser = async (id: string): Promise<void> => {
  await api.api.v1.users[':id'].$delete({
    param: { id },
  })
}

/**
 * 批量删除用户
 */
export const apiDeleteUserBatch = async (ids: string[]): Promise<void> => {
  await api.api.v1.users['batch-delete'].$post({
    json: { ids },
  })
}

/**
 * 更新用户状态
 */
export const apiUpdateUserStatus = async (id: string, status: UserStatus): Promise<void> => {
  await api.api.v1.users[':id'].status.$patch({
    param: { id },
    json: { status },
  })
}

/**
 * 重置用户密码
 */
export const apiResetUserPassword = async (id: string, password: string): Promise<void> => {
  await api.api.v1.users[':id'].password.$patch({
    param: { id },
    json: { password },
  })
}
