/**
 * UserManagement 模块的类型定义
 *
 * 使用 @seed/api-types 提供的共享类型，
 * 本文件仅定义 UI 层特有的状态和接口
 */

import type { Logger } from '@/core/logger.service'
import type { ErrorHandler } from '@/core/error.service'
import type {
  UserStatus,
  IUserVo,
  IUserCreateDto,
  IUserUpdateDto,
  IUserQueryDto,
  IPageResult,
} from '@seed/api-types'

// ============================================================================
// 重新导出 @seed/api-types 中的类型
// ============================================================================
export { UserStatus } from '@seed/api-types/enums'
export type {
  IUserVo,
  IUserCreateDto,
  IUserUpdateDto,
  IUserQueryDto,
  IPageResult,
} from '@seed/api-types'

// ============================================================================
// UI 层特有的类型定义
// ============================================================================

/**
 * 搜索表单数据（UI 层）
 */
export interface SearchFormData {
  username: string
  nickname: string
  status: UserStatus | undefined
}

/**
 * 用户表单数据（UI 层）
 */
export interface UserFormData {
  username: string
  password: string
  nickname: string
  email: string
  phone: string
}

/**
 * 重置密码表单数据（UI 层）
 */
export interface ResetPasswordFormData {
  password: string
  confirmPassword: string
}

/**
 * 分页状态（UI 层）
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
  userList: IUserVo[]
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

// ============================================================================
// Service 接口定义（依赖注入用）
// ============================================================================

/**
 * 用户管理 API 服务接口
 */
export interface UserManagementApiService {
  getPage(params: IUserQueryDto): Promise<IPageResult<IUserVo>>
  create(params: IUserCreateDto): Promise<IUserVo>
  update(id: number, params: IUserUpdateDto): Promise<IUserVo>
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
