/**
 * @file 应用全局状态
 * @description 管理应用级状态：主题、侧边栏、标题等
 * @module stores/app/app.store
 *
 * @exports useAppStore - Pinia Store，返回 { state, controller }
 *
 * @see ./app.types.ts - 类型定义（AppState, Theme）
 * @see ./app.controller.ts - 业务逻辑
 * @see ./app.service.ts - 服务实现
 */

import { reactive, markRaw } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { createLogger } from '@/core/logger.service'
import { AppController } from './app.controller'
import { BrowserThemeService } from './app.service'
import type { AppState, AppEnv } from './app.types'

// 组装环境依赖
const env: AppEnv = {
  logger: createLogger('App'),
  themeService: new BrowserThemeService('Seed Admin'),
}

/**
 * 应用全局状态管理
 *
 * 采用 Controller + Store 分层架构：
 * - Store: 负责响应式状态托管、HMR 适配
 * - Controller: 负责核心业务逻辑
 */
export const useAppStore = defineStore('app', () => {
  // 状态（响应式）
  const state = reactive<AppState>({
    title: 'Seed Admin',
    sidebarIsHidden: false,
    sidebarCollapsed: false,
    theme: 'light',
  })

  // Controller（使用 markRaw 避免响应式包装）
  const controller = markRaw(new AppController(state, env))

  return {
    state,
    controller,
  }
})

// 热模块替换支持
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useAppStore, import.meta.hot))
}
