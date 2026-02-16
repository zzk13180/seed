/**
 * @file 用户状态管理
 * @description 全局用户认证状态，包含登录/登出/Token 管理
 * @module stores/user/user.store
 *
 * @exports useUserStore - Pinia Store，返回 { state, controller, isLoggedIn }
 * @exports injectRouter - 注入 Router 实例（main.ts 中调用）
 *
 * @see ./user.types.ts - 类型定义
 * @see ./user.controller.ts - 业务逻辑
 * @see ./user.service.ts - 服务实现
 */

import { reactive, markRaw, computed } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { createLogger } from '@/core/logger.service'
import { LazyRouterNavigationService } from '@/core/navigation.service'
import { UserController } from './user.controller'
import { BetterAuthService, LocalStorageService } from './user.service'
import type { UserState, UserDeps } from './user.types'
import type { Router } from 'vue-router'

// router 实例引用（由 main.ts 注入）
let _router: Router | null = null

/**
 * 注入 router 实例（在 main.ts 中调用）
 */
export function injectRouter(router: Router): void {
  _router = router
}

/**
 * 获取 router 实例
 */
function getRouter(): Router {
  if (!_router) {
    throw new Error('Router not injected. Call injectRouter first in main.ts.')
  }
  return _router
}

/**
 * 创建依赖（使用懒加载导航服务避免循环依赖）
 */
function createDeps(): UserDeps {
  return {
    logger: createLogger('User'),
    authService: new BetterAuthService(),
    storageService: new LocalStorageService(),
    // 使用懒加载导航服务，router 在首次导航时才获取
    navigation: new LazyRouterNavigationService(() => getRouter()),
  }
}

/**
 * 用户状态管理
 *
 * 采用 Controller + Store 分层架构：
 * - Store: 负责响应式状态托管、计算属性
 * - Controller: 负责核心业务逻辑
 */
export const useUserStore = defineStore('user', () => {
  // 状态（响应式）
  const state = reactive<UserState>({
    user: null,
    loading: false,
  })

  // 延迟创建依赖，避免循环引用问题
  const deps = createDeps()

  // Controller（使用 markRaw 避免响应式包装）
  const controller = markRaw(new UserController(state, deps))

  // 计算属性（便于模板绑定）
  const isLoggedIn = computed(() => controller.isLoggedIn)
  const username = computed(() => controller.username)
  const nickname = computed(() => controller.nickname)
  const avatar = computed(() => controller.avatar)
  const role = computed(() => controller.role)
  const isAdmin = computed(() => controller.isAdmin)

  return {
    state,
    controller,
    // Getters
    isLoggedIn,
    username,
    nickname,
    avatar,
    role,
    isAdmin,
  }
})

// 热模块替换支持
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useUserStore, import.meta.hot))
}
