import { reactive, markRaw } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { createLogger } from '@/core/logger.service'
import { RouterNavigationService } from '@/core/navigation.service'
import { errorHandler } from '@/core/error.service'
import { router } from '@/pages/router'
import { HttpAuthService, LocalStorageService } from '@/stores/user/user.service'
import { LoginController } from './login.controller'
import { ElementMessageService } from './login.service'
import type { LoginState, LoginEnv } from './login.types'

// 组装环境依赖
const env: LoginEnv = {
  logger: createLogger('Login'),
  authService: new HttpAuthService(),
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
      username: 'admin',
      password: 'admin123',
      rememberMe: false,
    },
    loading: false,
    errorMessage: null,
  })

  // Controller（使用 markRaw 避免响应式包装）
  const controller = markRaw(new LoginController(state, env))

  return {
    state,
    controller,
  }
})

// 热模块替换支持
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useLoginStore, import.meta.hot))
}
