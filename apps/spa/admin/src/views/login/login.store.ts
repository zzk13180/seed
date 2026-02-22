import { reactive, markRaw } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { createLogger } from '@/core/logger.service'
import { RouterNavigationService } from '@/core/navigation.service'
import { errorHandler } from '@/core/error.service'
import { router } from '@/pages/router'
import { BetterAuthService, LocalStorageService } from '@/stores/user/user.service'
import { LoginController } from './login.controller'
import { ElementMessageService } from './login.service'
import type { LoginState, LoginDeps } from './login.types'

// 组装依赖
const deps: LoginDeps = {
  logger: createLogger('Login'),
  authService: new BetterAuthService(),
  storageService: new LocalStorageService(),
  navigation: new RouterNavigationService(router),
  messageService: new ElementMessageService(),
  errorHandler,
}

/**
 * 登录状态管理
 *
 * 采用 Controller + Store 分层架构：
 * - Store: 负责响应式状态托管
 * - Controller: 负责核心业务逻辑
 */
export const useLoginStore = defineStore('login', () => {
  // 状态（响应式）
  const state = reactive<LoginState>({
    form: {
      email: 'admin@seed.dev',
      password: 'admin123',
      rememberMe: false,
    },
    loading: false,
    errorMessage: null,
  })

  // Controller（使用 markRaw 避免响应式包装）
  const controller = markRaw(new LoginController(state, deps))

  return {
    state,
    controller,
  }
})

// 热模块替换支持
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useLoginStore, import.meta.hot))
}
