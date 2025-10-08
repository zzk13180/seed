/**
 * App 模块的类型定义
 * 纯 TypeScript 类型，不依赖任何框架
 */

import type { Logger } from '@/core/logger.service'

/** 主题类型 */
export type Theme = 'light' | 'dark' | 'system'

/**
 * 应用状态接口
 */
export interface AppState {
  /** 页面标题 */
  title: string
  /** 侧边栏是否隐藏 */
  sidebarIsHidden: boolean
  /** 侧边栏是否折叠 */
  sidebarCollapsed: boolean
  /** 当前主题 */
  theme: Theme
}

/**
 * 主题服务接口 - 抽象 DOM 操作
 */
export interface ThemeService {
  applyTheme(theme: Theme): void
  setDocumentTitle(title: string): void
}

/**
 * App 模块环境依赖
 */
export interface AppEnv {
  logger: Logger
  themeService: ThemeService
}
