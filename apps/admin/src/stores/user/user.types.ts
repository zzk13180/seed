/**
 * User 模块的类型定义
 *
 * 使用 @seed/api-types 共享类型，本文件定义 UI 层特有类型
 */

import type { Logger } from '@/core/logger.service'
import type { NavigationService } from '@/core/navigation.service'
import type { IUserVo, ILoginDto, ILoginVo } from '@seed/api-types'

// ============================================================================
// 从 @seed/api-types 重新导出的类型
// ============================================================================
export { UserStatus } from '@seed/api-types/enums'
export type { IUserVo, ILoginDto, ILoginVo } from '@seed/api-types'

// ============================================================================
// UI 层特有的类型定义
// ============================================================================

/**
 * 用户状态接口
 */
export interface UserState {
  /** 当前用户信息 */
  user: IUserVo | null
  /** 是否正在加载 */
  loading: boolean
}

/**
 * 认证服务接口 - 抽象 API 调用
 */
export interface AuthService {
  login(params: ILoginDto): Promise<ILoginVo>
  logout(): Promise<void>
  getCurrentUser(): Promise<IUserVo>
  refreshToken(): Promise<ILoginVo>
  isAuthenticated(): boolean
  initAuth(): boolean
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
 * User 模块环境依赖
 */
export interface UserEnv {
  logger: Logger
  authService: AuthService
  storageService: StorageService
  navigation: NavigationService
}
