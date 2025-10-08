import { reactive, markRaw, computed } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { createLogger } from '@/core/logger.service'
import { errorHandler } from '@/core/error.service'
import { UserManagementController } from './user-management.controller'
import {
  HttpUserManagementApiService,
  ElementMessageService,
  ElementConfirmService,
} from './user-management.service'
import type { UserManagementState, UserManagementEnv } from './user-management.types'

// 组装环境依赖
const env: UserManagementEnv = {
  logger: createLogger('UserManagement'),
  apiService: new HttpUserManagementApiService(),
  messageService: new ElementMessageService(),
  confirmService: new ElementConfirmService(),
  errorHandler,
}

/**
 * 用户管理状态
 *
 * 采用 Controller + Store 分层架构：
 * - Store: 负责响应式状态托管、计算属性
 * - Controller: 负责核心业务逻辑
 */
export const useUserManagementStore = defineStore('userManagement', () => {
  // 状态（响应式）
  const state = reactive<UserManagementState>({
    userList: [],
    loading: false,
    selectedIds: [],
    pagination: {
      page: 1,
      pageSize: 10,
      total: 0,
    },
    searchForm: {
      username: '',
      nickname: '',
      status: undefined,
    },
    userForm: {
      username: '',
      password: '',
      nickname: '',
      email: '',
      phone: '',
    },
    resetForm: {
      password: '',
      confirmPassword: '',
    },
    dialogVisible: false,
    isEdit: false,
    currentUserId: null,
    submitLoading: false,
    resetPasswordVisible: false,
    resetUserId: null,
    resetLoading: false,
    errorMessage: null,
  })

  // Controller（使用 markRaw 避免响应式包装）
  const controller = markRaw(new UserManagementController(state, env))

  // 计算属性
  const dialogTitle = computed(() => controller.dialogTitle)
  const canBatchDelete = computed(() => controller.canBatchDelete)

  return {
    state,
    controller,
    // Getters
    dialogTitle,
    canBatchDelete,
  }
})

// 热模块替换支持
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useUserManagementStore, import.meta.hot))
}
