/**
 * UserManagementController 单元测试 — 前端 Controller 层测试示例
 *
 * 测试策略：
 * - 所有依赖（API / Message / Confirm / Logger）通过构造函数注入 mock
 * - 直接操作 state 对象验证状态变更
 * - Controller 是纯 TypeScript 类，无需 Vue / DOM 环境
 *
 * 覆盖模式：
 * - 列表加载 + loading 状态管理
 * - 搜索 / 重置重置分页
 * - 弹窗开关 + 表单状态
 * - 提交成功 / 失败路径
 * - 确认对话框交互
 */
import { describe, it, expect, vi, beforeEach } from 'vitest'
import { UserManagementController } from '../user-management.controller'
import { UserStatus } from '../user-management.types'
import type {
  UserManagementState,
  UserManagementDeps,
  UserManagementApiService,
  MessageService,
  ConfirmService,
} from '../user-management.types'
import type { UserVO } from '@seed/contracts'

// ─── Mock Factories ──────────────────────────────────────────────────────────
function createMockState(): UserManagementState {
  return {
    userList: [],
    loading: false,
    selectedIds: [],
    pagination: { page: 1, pageSize: 10, total: 0 },
    searchForm: { username: '', nickname: '', status: undefined },
    userForm: { username: '', password: '', nickname: '', email: '', phone: '' },
    resetForm: { password: '', confirmPassword: '' },
    dialogVisible: false,
    isEdit: false,
    currentUserId: null,
    submitLoading: false,
    resetPasswordVisible: false,
    resetUserId: null,
    resetLoading: false,
    errorMessage: null,
  }
}

function createMockDeps(): UserManagementDeps {
  return {
    logger: { debug: vi.fn(), info: vi.fn(), warn: vi.fn(), error: vi.fn() },
    apiService: {
      getPage: vi.fn(),
      create: vi.fn(),
      update: vi.fn(),
      delete: vi.fn(),
      deleteBatch: vi.fn(),
      updateStatus: vi.fn(),
      resetPassword: vi.fn(),
    } as UserManagementApiService,
    messageService: {
      success: vi.fn(),
      error: vi.fn(),
      warning: vi.fn(),
      info: vi.fn(),
    } as MessageService,
    confirmService: {
      confirm: vi.fn(),
    } as ConfirmService,
    errorHandler: {
      parseError: vi.fn(),
      getErrorType: vi.fn(),
      getUserMessage: vi.fn(),
    } as any,
  }
}

const MOCK_USER: UserVO = {
  id: 'user-1',
  name: 'Test User',
  email: 'test@example.com',
  emailVerified: true,
  image: null,
  phone: null,
  role: 'user',
  status: UserStatus.ENABLED,
  createdAt: '2025-01-15T10:00:00.000Z',
  updatedAt: '2025-01-15T10:00:00.000Z',
}

// ─── Tests ───────────────────────────────────────────────────────────────────
describe('UserManagementController', () => {
  let state: UserManagementState
  let deps: UserManagementDeps
  let controller: UserManagementController

  beforeEach(() => {
    state = createMockState()
    deps = createMockDeps()
    controller = new UserManagementController(state, deps)
  })

  // ── Computed Properties ────────────────────────────────────────────────
  describe('computed', () => {
    it('dialogTitle should reflect edit/create mode', () => {
      state.isEdit = false
      expect(controller.dialogTitle).toBe('新增用户')

      state.isEdit = true
      expect(controller.dialogTitle).toBe('编辑用户')
    })

    it('canBatchDelete should depend on selectedIds', () => {
      expect(controller.canBatchDelete).toBe(false)

      state.selectedIds = ['id-1', 'id-2']
      expect(controller.canBatchDelete).toBe(true)
    })
  })

  // ── fetchUserList ──────────────────────────────────────────────────────
  describe('fetchUserList', () => {
    it('should set loading=true, fetch data, then loading=false', async () => {
      ;(deps.apiService.getPage as any).mockResolvedValue({
        list: [MOCK_USER],
        total: 1,
      })

      const promise = controller.fetchUserList()

      // loading 应立即为 true
      expect(state.loading).toBe(true)

      await promise

      // 完成后 loading 恢复，数据已填充
      expect(state.loading).toBe(false)
      expect(state.userList).toEqual([MOCK_USER])
      expect(state.pagination.total).toBe(1)
    })

    it('should show error message on API failure', async () => {
      ;(deps.apiService.getPage as any).mockRejectedValue(new Error('Network Error'))

      await controller.fetchUserList()

      expect(state.loading).toBe(false)
      expect(deps.messageService.error).toHaveBeenCalledWith('获取用户列表失败')
    })
  })

  // ── handleSearch / handleReset ─────────────────────────────────────────
  describe('search & reset', () => {
    it('handleSearch should reset page to 1', async () => {
      ;(deps.apiService.getPage as any).mockResolvedValue({ list: [], total: 0 })
      state.pagination.page = 5

      await controller.handleSearch()

      expect(state.pagination.page).toBe(1)
      expect(deps.apiService.getPage).toHaveBeenCalled()
    })

    it('handleReset should clear search form and reset page', async () => {
      ;(deps.apiService.getPage as any).mockResolvedValue({ list: [], total: 0 })
      state.searchForm.username = 'test'
      state.searchForm.status = UserStatus.ENABLED
      state.pagination.page = 3

      await controller.handleReset()

      expect(state.searchForm.username).toBe('')
      expect(state.searchForm.status).toBeUndefined()
      expect(state.pagination.page).toBe(1)
    })
  })

  // ── Dialog Management ──────────────────────────────────────────────────
  describe('dialog', () => {
    it('openCreateDialog should reset form and open dialog', () => {
      state.userForm.username = 'old'

      controller.openCreateDialog()

      expect(state.dialogVisible).toBe(true)
      expect(state.isEdit).toBe(false)
      expect(state.userForm.username).toBe('')
    })

    it('openEditDialog should populate form from user data', () => {
      controller.openEditDialog(MOCK_USER)

      expect(state.dialogVisible).toBe(true)
      expect(state.isEdit).toBe(true)
      expect(state.currentUserId).toBe('user-1')
      expect(state.userForm.username).toBe('Test User')
      expect(state.userForm.email).toBe('test@example.com')
    })
  })

  // ── handleSubmit ───────────────────────────────────────────────────────
  describe('handleSubmit', () => {
    it('should create user in create mode', async () => {
      ;(deps.apiService.create as any).mockResolvedValue(MOCK_USER)
      ;(deps.apiService.getPage as any).mockResolvedValue({ list: [MOCK_USER], total: 1 })

      state.isEdit = false
      state.userForm.username = 'newuser'
      state.userForm.password = 'pass123'

      const result = await controller.handleSubmit()

      expect(result).toBe(true)
      expect(deps.apiService.create).toHaveBeenCalledWith(
        expect.objectContaining({ username: 'newuser', password: 'pass123' }),
      )
      expect(deps.messageService.success).toHaveBeenCalledWith('创建成功')
      expect(state.dialogVisible).toBe(false)
    })

    it('should update user in edit mode', async () => {
      ;(deps.apiService.update as any).mockResolvedValue(MOCK_USER)
      ;(deps.apiService.getPage as any).mockResolvedValue({ list: [MOCK_USER], total: 1 })

      state.isEdit = true
      state.currentUserId = 'user-1'
      state.userForm.nickname = 'New Nickname'

      const result = await controller.handleSubmit()

      expect(result).toBe(true)
      expect(deps.apiService.update).toHaveBeenCalledWith(
        'user-1',
        expect.objectContaining({ nickname: 'New Nickname' }),
      )
      expect(deps.messageService.success).toHaveBeenCalledWith('更新成功')
    })

    it('should return false and show error on failure', async () => {
      ;(deps.apiService.create as any).mockRejectedValue(new Error('Server Error'))

      state.isEdit = false
      const result = await controller.handleSubmit()

      expect(result).toBe(false)
      expect(deps.messageService.error).toHaveBeenCalledWith('Server Error')
      expect(state.submitLoading).toBe(false)
    })
  })

  // ── handleDelete ───────────────────────────────────────────────────────
  describe('handleDelete', () => {
    it('should delete after user confirms', async () => {
      ;(deps.confirmService.confirm as any).mockResolvedValue(true)
      ;(deps.apiService.delete as any).mockResolvedValue(undefined)
      ;(deps.apiService.getPage as any).mockResolvedValue({ list: [], total: 0 })

      await controller.handleDelete(MOCK_USER)

      expect(deps.confirmService.confirm).toHaveBeenCalledWith(
        expect.stringContaining('Test User'),
        '提示',
      )
      expect(deps.apiService.delete).toHaveBeenCalledWith('user-1')
      expect(deps.messageService.success).toHaveBeenCalledWith('删除成功')
    })

    it('should not delete when user cancels', async () => {
      ;(deps.confirmService.confirm as any).mockResolvedValue(false)

      await controller.handleDelete(MOCK_USER)

      expect(deps.apiService.delete).not.toHaveBeenCalled()
    })
  })

  // ── handleStatusChange ─────────────────────────────────────────────────
  describe('handleStatusChange', () => {
    it('should revert status on API failure', async () => {
      ;(deps.apiService.updateStatus as any).mockRejectedValue(new Error('fail'))

      const user = { ...MOCK_USER, status: UserStatus.ENABLED }
      await controller.handleStatusChange(user)

      // 状态应被回滚
      expect(user.status).toBe(UserStatus.DISABLED)
      expect(deps.messageService.error).toHaveBeenCalledWith('状态更新失败')
    })
  })
})
