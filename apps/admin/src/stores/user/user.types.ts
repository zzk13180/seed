/**
 * User 模块的类型定义
 *
 * 使用 @seed/kit 共享类型，本文件定义 UI 层特有类型
 */

import type { Logger } from '@/core/logger.service'
import type { NavigationService } from '@/core/navigation.service'
import type { UserVO } from '@seed/contracts'

// ============================================================================
// 从 @seed/contracts 重新导出的类型
// ============================================================================
export { UserStatus, type UserVO } from '@seed/contracts'

// ============================================================================
// UI 层特有的类型定义
// ============================================================================

/**
 * 登录参数
 */
export interface LoginParams {
  email: string
  password: string
}

/**
 * 用户状态接口
 */
export interface UserState {
  /** 当前用户信息 */
  user: UserVO | null
  /** 是否正在加载 */
  loading: boolean
}

/**
 * 认证服务接口 - 抽象 API 调用
 *
 * Better Auth 使用 cookie-session 模式:
 * - 登录成功后浏览器自动携带 session cookie
 * - 无需手动管理 token
 */
export interface AuthService {
  login(params: LoginParams): Promise<UserVO>
  logout(): Promise<void>
  getCurrentUser(): Promise<UserVO | null>
  isAuthenticated(): Promise<boolean>
}

/**
 * 本地存储服务接口 - 抽象存储操作
 */
export interface StorageService {
  get(key: string): string | null
  set(key: string, value: string): void
  remove(key: string): void
}

/**
 * User 模块依赖
 */
export interface UserDeps {
  logger: Logger
  authService: AuthService
  storageService: StorageService
  navigation: NavigationService
}
