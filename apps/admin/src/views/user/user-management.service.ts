import { ElMessage, ElMessageBox } from 'element-plus'
import {
  apiGetUserPage,
  apiCreateUser,
  apiUpdateUser,
  apiDeleteUser,
  apiDeleteUserBatch,
  apiUpdateUserStatus,
  apiResetUserPassword,
} from '@/api/user.api'
import type {
  UserManagementApiService,
  MessageService,
  ConfirmService,
  UserQueryParams,
  UserCreateParams,
  UserUpdateParams,
  UserInfo,
  PageResult,
  UserStatus,
} from './user-management.types'

/**
 * HTTP 用户管理 API 服务实现
 */
export class HttpUserManagementApiService implements UserManagementApiService {
  async getPage(params: UserQueryParams): Promise<PageResult<UserInfo>> {
    return apiGetUserPage(params)
  }

  async create(params: UserCreateParams): Promise<UserInfo> {
    return apiCreateUser(params)
  }

  async update(id: number, params: UserUpdateParams): Promise<UserInfo> {
    return apiUpdateUser(id, params)
  }

  async delete(id: number): Promise<void> {
    await apiDeleteUser(id)
  }

  async deleteBatch(ids: number[]): Promise<void> {
    await apiDeleteUserBatch(ids)
  }

  async updateStatus(id: number, status: UserStatus): Promise<void> {
    await apiUpdateUserStatus(id, status)
  }

  async resetPassword(id: number, password: string): Promise<void> {
    await apiResetUserPassword(id, password)
  }
}

/**
 * Element Plus 消息服务实现
 */
export class ElementMessageService implements MessageService {
  success(message: string): void {
    ElMessage.success(message)
  }

  error(message: string): void {
    ElMessage.error(message)
  }

  warning(message: string): void {
    ElMessage.warning(message)
  }

  info(message: string): void {
    ElMessage.info(message)
  }
}

/**
 * Element Plus 确认对话框服务实现
 */
export class ElementConfirmService implements ConfirmService {
  async confirm(message: string, title: string = '提示'): Promise<boolean> {
    try {
      await ElMessageBox.confirm(message, title, { type: 'warning' })
      return true
    } catch {
      return false
    }
  }
}
