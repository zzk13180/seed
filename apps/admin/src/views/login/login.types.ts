/**
 * Login 模块的类型定义
 *
 * 使用 @seed/api-types 共享类型，本文件定义 UI 层特有类型
 */

import type { Logger } from '@/core/logger.service'
import type { NavigationService } from '@/core/navigation.service'
import type { ErrorHandler } from '@/core/error.service'
import type { AuthService, StorageService } from '@/stores/user/user.types'

// ============================================================================
// 从 @seed/api-types 重新导出的类型
// ============================================================================
export type { ILoginDto, IUserVo } from '@seed/api-types'

// ============================================================================
// UI 层特有的类型定义
// ============================================================================

/**
 * 登录表单数据
 */
export interface LoginFormData {
  username: string
  password: string
  rememberMe: boolean
}

/**
 * 登录状态
 */
export interface LoginState {
  /** 表单数据 */
  form: LoginFormData
  /** 是否正在加载 */
  loading: boolean
  /** 错误信息 */
  errorMessage: string | null
}

/**
 * 表单验证结果
 */
export interface ValidationResult {
  valid: boolean
  errors: {
    username?: string
    password?: string
  }
}

/**
 * 消息服务接口 - 抽象 UI 消息提示
 */
export interface MessageService {
  success(message: string): void
  error(message: string): void
  warning(message: string): void
  info(message: string): void
}

/**
 * Login 模块环境依赖
 */
export interface LoginEnv {
  logger: Logger
  authService: AuthService
  storageService: StorageService
  navigation: NavigationService
  messageService: MessageService
  errorHandler: ErrorHandler
}
