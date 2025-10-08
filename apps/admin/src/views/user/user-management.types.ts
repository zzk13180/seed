/**
 * UserManagement 模块的类型定义
 * 纯 TypeScript 类型，不依赖任何框架
 */

import type { Logger } from '@/core/logger.service'
import type { ErrorHandler } from '@/core/error.service'

/**
 * 用户状态枚举
 */
export enum UserStatus {
  DISABLED = 0,
  ENABLED = 1,
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
 * 用户查询参数
 */
export interface UserQueryParams {
  username?: string
  nickname?: string
  email?: string
  phone?: string
  status?: UserStatus
  page?: number
  pageSize?: number
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
 * 分页结果
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
 * 搜索表单数据
 */
export interface SearchFormData {
  username: string
  nickname: string
  status: UserStatus | undefined
}

/**
 * 用户表单数据
 */
export interface UserFormData {
  username: string
  password: string
  nickname: string
  email: string
  phone: string
}

/**
 * 重置密码表单数据
 */
export interface ResetPasswordFormData {
  password: string
  confirmPassword: string
}

/**
 * 分页状态
 */
export interface PaginationState {
  page: number
  pageSize: number
  total: number
}

/**
 * UserManagement 状态
 */
export interface UserManagementState {
  /** 用户列表 */
  userList: UserInfo[]
  /** 是否正在加载 */
  loading: boolean
  /** 选中的用户 ID 列表 */
  selectedIds: number[]
  /** 分页状态 */
  pagination: PaginationState
  /** 搜索表单 */
  searchForm: SearchFormData
  /** 用户表单 */
  userForm: UserFormData
  /** 重置密码表单 */
  resetForm: ResetPasswordFormData
  /** 是否显示用户弹窗 */
  dialogVisible: boolean
  /** 是否为编辑模式 */
  isEdit: boolean
  /** 当前编辑的用户 ID */
  currentUserId: number | null
  /** 表单提交中 */
  submitLoading: boolean
  /** 是否显示重置密码弹窗 */
  resetPasswordVisible: boolean
  /** 重置密码的用户 ID */
  resetUserId: number | null
  /** 重置密码提交中 */
  resetLoading: boolean
  /** 错误信息 */
  errorMessage: string | null
}

/**
 * 用户管理 API 服务接口
 */
export interface UserManagementApiService {
  getPage(params: UserQueryParams): Promise<PageResult<UserInfo>>
  create(params: UserCreateParams): Promise<UserInfo>
  update(id: number, params: UserUpdateParams): Promise<UserInfo>
  delete(id: number): Promise<void>
  deleteBatch(ids: number[]): Promise<void>
  updateStatus(id: number, status: UserStatus): Promise<void>
  resetPassword(id: number, password: string): Promise<void>
}

/**
 * 消息服务接口
 */
export interface MessageService {
  success(message: string): void
  error(message: string): void
  warning(message: string): void
  info(message: string): void
}

/**
 * 确认对话框服务接口
 */
export interface ConfirmService {
  confirm(message: string, title?: string): Promise<boolean>
}

/**
 * UserManagement 模块环境依赖
 */
export interface UserManagementEnv {
  logger: Logger
  apiService: UserManagementApiService
  messageService: MessageService
  confirmService: ConfirmService
  errorHandler: ErrorHandler
}
