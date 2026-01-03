import { UserStatus } from './user-management.types'
import type { IUserVo, IUserCreateDto, IUserUpdateDto } from '@seed/api-types'
import type { UserManagementState, UserManagementEnv } from './user-management.types'

/**
 * UserManagementController - 纯 TypeScript 类，不依赖任何框架
 *
 * 职责：
 * - 管理用户列表 CRUD 操作
 * - 处理分页、搜索逻辑
 * - 表单状态管理
 * - 通过 Env 注入执行副作用
 */
export class UserManagementController {
  private readonly state: UserManagementState
  private readonly env: UserManagementEnv

  constructor(state: UserManagementState, env: UserManagementEnv) {
    this.state = state
    this.env = env
  }

  /**
   * 弹窗标题
   */
  get dialogTitle(): string {
    return this.state.isEdit ? '编辑用户' : '新增用户'
  }

  /**
   * 是否可以批量删除
   */
  get canBatchDelete(): boolean {
    return this.state.selectedIds.length > 0
  }

  /**
   * 获取用户列表
   */
  async fetchUserList(): Promise<void> {
    this.state.loading = true
    this.env.logger.debug('Fetching user list')

    try {
      const result = await this.env.apiService.getPage({
        ...this.state.searchForm,
        page: this.state.pagination.page,
        pageSize: this.state.pagination.pageSize,
      })

      this.state.userList = result.list
      this.state.pagination.total = result.total
      this.env.logger.debug('User list fetched', { total: result.total })
    } catch (error) {
      this.env.logger.error('Failed to fetch user list', error)
      this.env.messageService.error('获取用户列表失败')
    } finally {
      this.state.loading = false
    }
  }

  /**
   * 搜索
   */
  async handleSearch(): Promise<void> {
    this.state.pagination.page = 1
    await this.fetchUserList()
  }

  /**
   * 重置搜索
   */
  async handleReset(): Promise<void> {
    this.state.searchForm.username = ''
    this.state.searchForm.nickname = ''
    this.state.searchForm.status = undefined
    this.state.pagination.page = 1
    await this.fetchUserList()
  }

  /**
   * 分页大小变化
   */
  async handleSizeChange(size: number): Promise<void> {
    this.state.pagination.pageSize = size
    await this.fetchUserList()
  }

  /**
   * 页码变化
   */
  async handlePageChange(page: number): Promise<void> {
    this.state.pagination.page = page
    await this.fetchUserList()
  }

  /**
   * 选择变化
   */
  handleSelectionChange(selection: IUserVo[]): void {
    this.state.selectedIds = selection.map(item => item.id)
  }

  /**
   * 打开新增弹窗
   */
  openCreateDialog(): void {
    this.state.isEdit = false
    this.state.currentUserId = null
    this.resetUserForm()
    this.state.dialogVisible = true
  }

  /**
   * 打开编辑弹窗
   */
  openEditDialog(user: IUserVo): void {
    this.state.isEdit = true
    this.state.currentUserId = user.id
    this.state.userForm.username = user.username
    this.state.userForm.password = ''
    this.state.userForm.nickname = user.nickname || ''
    this.state.userForm.email = user.email || ''
    this.state.userForm.phone = user.phone || ''
    this.state.dialogVisible = true
  }

  /**
   * 关闭用户弹窗
   */
  closeDialog(): void {
    this.state.dialogVisible = false
  }

  /**
   * 提交表单
   */
  async handleSubmit(): Promise<boolean> {
    this.state.submitLoading = true
    this.env.logger.debug('Submitting user form', { isEdit: this.state.isEdit })

    try {
      if (this.state.isEdit && this.state.currentUserId) {
        const updateParams: IUserUpdateDto = {
          nickname: this.state.userForm.nickname || undefined,
          email: this.state.userForm.email || undefined,
          phone: this.state.userForm.phone || undefined,
        }
        await this.env.apiService.update(this.state.currentUserId, updateParams)
        this.env.messageService.success('更新成功')
      } else {
        const createParams: IUserCreateDto = {
          username: this.state.userForm.username,
          password: this.state.userForm.password,
          nickname: this.state.userForm.nickname || undefined,
          email: this.state.userForm.email || undefined,
          phone: this.state.userForm.phone || undefined,
        }
        await this.env.apiService.create(createParams)
        this.env.messageService.success('创建成功')
      }

      this.state.dialogVisible = false
      await this.fetchUserList()
      return true
    } catch (error) {
      this.env.logger.error('Submit failed', error)
      this.env.messageService.error(error instanceof Error ? error.message : '操作失败')
      return false
    } finally {
      this.state.submitLoading = false
    }
  }

  /**
   * 删除用户
   */
  async handleDelete(user: IUserVo): Promise<void> {
    try {
      const confirmed = await this.env.confirmService.confirm(
        `确定要删除用户 "${user.username}" 吗？`,
        '提示',
      )
      if (!confirmed) return

      await this.env.apiService.delete(user.id)
      this.env.messageService.success('删除成功')
      await this.fetchUserList()
    } catch (error) {
      this.env.logger.error('Delete failed', error)
      this.env.messageService.error('删除失败')
    }
  }

  /**
   * 批量删除
   */
  async handleBatchDelete(): Promise<void> {
    if (this.state.selectedIds.length === 0) return

    try {
      const confirmed = await this.env.confirmService.confirm(
        `确定要删除选中的 ${this.state.selectedIds.length} 个用户吗？`,
        '提示',
      )
      if (!confirmed) return

      await this.env.apiService.deleteBatch(this.state.selectedIds)
      this.env.messageService.success('删除成功')
      this.state.selectedIds = []
      await this.fetchUserList()
    } catch (error) {
      this.env.logger.error('Batch delete failed', error)
      this.env.messageService.error('批量删除失败')
    }
  }

  /**
   * 状态变化
   */
  async handleStatusChange(user: IUserVo): Promise<void> {
    try {
      await this.env.apiService.updateStatus(user.id, user.status)
      this.env.messageService.success('状态更新成功')
    } catch (error) {
      this.env.logger.error('Status update failed', error)
      this.env.messageService.error('状态更新失败')
      // 恢复原状态
      user.status = user.status === UserStatus.ENABLED ? UserStatus.DISABLED : UserStatus.ENABLED
    }
  }

  /**
   * 打开重置密码弹窗
   */
  openResetPasswordDialog(user: IUserVo): void {
    this.state.resetUserId = user.id
    this.state.resetForm.password = ''
    this.state.resetForm.confirmPassword = ''
    this.state.resetPasswordVisible = true
  }

  /**
   * 关闭重置密码弹窗
   */
  closeResetPasswordDialog(): void {
    this.state.resetPasswordVisible = false
  }

  /**
   * 提交重置密码
   */
  async handleResetSubmit(): Promise<boolean> {
    if (!this.state.resetUserId) return false

    this.state.resetLoading = true

    try {
      await this.env.apiService.resetPassword(this.state.resetUserId, this.state.resetForm.password)
      this.env.messageService.success('密码重置成功')
      this.state.resetPasswordVisible = false
      return true
    } catch (error) {
      this.env.logger.error('Reset password failed', error)
      this.env.messageService.error('密码重置失败')
      return false
    } finally {
      this.state.resetLoading = false
    }
  }

  /**
   * 格式化日期
   */
  formatDate(dateStr: string): string {
    if (!dateStr) return '-'
    try {
      const date = new Date(dateStr)
      return date.toLocaleString('zh-CN', {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
      })
    } catch {
      return dateStr
    }
  }

  /**
   * 重置用户表单
   */
  private resetUserForm(): void {
    this.state.userForm.username = ''
    this.state.userForm.password = ''
    this.state.userForm.nickname = ''
    this.state.userForm.email = ''
    this.state.userForm.phone = ''
  }
}
