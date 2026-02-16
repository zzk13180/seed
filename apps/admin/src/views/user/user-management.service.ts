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
  UserVO,
  UserQuery as UserQueryDto,
  UserCreateDto,
  UserUpdateDto,
  PageResult,
} from '@seed/contracts'
import type { UserStatus } from '@seed/contracts'
import type {
  UserManagementApiService,
  MessageService,
  ConfirmService,
} from './user-management.types'

/**
 * HTTP 用户管理 API 服务实现
 */
export class HttpUserManagementApiService implements UserManagementApiService {
  async getPage(params: UserQueryDto): Promise<PageResult<UserVO>> {
    return apiGetUserPage(params)
  }

  async create(params: UserCreateDto): Promise<UserVO> {
    return apiCreateUser(params)
  }

  async update(id: string, params: UserUpdateDto): Promise<UserVO> {
    return apiUpdateUser(id, params)
  }

  async delete(id: string): Promise<void> {
    await apiDeleteUser(id)
  }

  async deleteBatch(ids: string[]): Promise<void> {
    await apiDeleteUserBatch(ids)
  }

  async updateStatus(id: string, status: UserStatus): Promise<void> {
    await apiUpdateUserStatus(id, status)
  }

  async resetPassword(id: string, password: string): Promise<void> {
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
