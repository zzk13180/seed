/**
 * User 模块的类型定义
 * 纯 TypeScript 类型，不依赖任何框架
 */

import type { Logger } from '@/core/logger.service'
import type { NavigationService } from '@/core/navigation.service'

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
 * 登录参数
 */
export interface LoginParams {
  username: string
  password: string
}

/**
 * 登录结果
 */
export interface LoginResult {
  accessToken: string
  refreshToken: string
  user: UserInfo
  expiresIn: number
}

/**
 * 用户状态接口
 */
export interface UserState {
  /** 当前用户信息 */
  user: UserInfo | null
  /** 是否正在加载 */
  loading: boolean
}

/**
 * 认证服务接口 - 抽象 API 调用
 */
export interface AuthService {
  login(params: LoginParams): Promise<LoginResult>
  logout(): Promise<void>
  getCurrentUser(): Promise<UserInfo>
  refreshToken(): Promise<LoginResult>
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
