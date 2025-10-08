/**
 * API 类型定义
 */

/**
 * 用户状态枚举
 */
export enum UserStatus {
  DISABLED = 0,
  ENABLED = 1,
}

/**
 * 统一响应格式
 */
export interface ApiResponse<T = unknown> {
  code: number
  data: T
  message: string
  timestamp: number
  traceId: string | null
}

/**
 * 分页请求参数
 */
export interface PageRequest {
  page?: number
  pageSize?: number
  orderBy?: string
  orderDirection?: 'ASC' | 'DESC'
  keyword?: string
}

/**
 * 分页响应
 */
export interface PageResult<T> {
  list: T[]
  total: number
  page: number
  pageSize: number
  totalPages: number
  hasNext: boolean
  hasPrevious: boolean
}

/**
 * 用户信息
 */
export interface UserInfo {
  id: number
  username: string
  nickname: string | null
  email: string | null
  phone: string | null
  avatar: string | null
  status: UserStatus
  createdAt: string
  updatedAt: string
}

/**
 * 登录响应
 */
export interface LoginResult {
  accessToken: string
  refreshToken: string
  user: UserInfo
  expiresIn: number
}

/**
 * 登录请求参数
 */
export interface LoginParams {
  username: string
  password: string
}

/**
 * 用户查询参数
 */
export interface UserQueryParams extends PageRequest {
  username?: string
  nickname?: string
  email?: string
  phone?: string
  status?: UserStatus
}

/**
 * 用户创建参数
 */
export interface UserCreateParams {
  username: string
  password: string
  nickname?: string
  email?: string
  phone?: string
  avatar?: string
}

/**
 * 用户更新参数
 */
export interface UserUpdateParams {
  nickname?: string
  email?: string
  phone?: string
  avatar?: string
}

/**
 * 健康检查响应
 */
export interface HealthCheckResult {
  status: 'ok' | 'error'
  info?: Record<string, unknown>
  error?: Record<string, unknown>
  details?: Record<string, unknown>
}
