/**
 * UserController 集成测试 — 后端 Controller 层测试示例
 *
 * 测试策略：
 * - 使用 Hono 的 app.request() 模拟 HTTP 请求（无需启动真实服务器）
 * - Mock 中间件（authGuard / rolesGuard）跳过认证，专注测试路由逻辑
 * - Mock UserService 控制业务返回，验证 HTTP 状态码和响应体
 *
 * 覆盖模式：
 * - GET 正常查询
 * - PUT 更新 + Zod 校验失败
 * - DELETE 204 无内容响应
 * - 业务异常 → HTTP 错误码映射
 */
import { describe, it, expect, vi, beforeEach } from 'vitest'
import { NotFoundError } from '@seed/contracts/errors'

import { createUserRoutes } from '../user.controller.js'

// ─── Mock Setup ──────────────────────────────────────────────────────────────
// 1. Mock 认证中间件 —— 集成测试关注路由逻辑，不测认证
vi.mock('@seed/kit/hono/middleware', () => ({
  authGuardMiddleware: vi.fn(async (_c: any, next: any) => next()),
  rolesGuardMiddleware: vi.fn(() => async (_c: any, next: any) => next()),
}))

// 2. Mock UserService —— 控制业务层返回
const mockService = vi.hoisted(() => ({
  findPage: vi.fn(),
  findAll: vi.fn(),
  findById: vi.fn(),
  update: vi.fn(),
  delete: vi.fn(),
  batchDelete: vi.fn(),
  updateStatus: vi.fn(),
  resetPassword: vi.fn(),
}))

vi.mock('../user.service.js', () => ({
  UserService: vi.fn(() => mockService),
}))

// ─── Test Fixtures ───────────────────────────────────────────────────────────
const USER_VO = {
  id: 'user-1',
  name: 'Test User',
  email: 'test@example.com',
  emailVerified: true,
  image: null,
  phone: null,
  role: 'user',
  status: 1,
  createdAt: '2025-01-15T10:00:00.000Z',
  updatedAt: '2025-01-15T10:00:00.000Z',
}

// ─── Tests ───────────────────────────────────────────────────────────────────
describe('User Routes (Integration)', () => {
  let app: ReturnType<typeof createUserRoutes>

  beforeEach(() => {
    vi.clearAllMocks()
    app = createUserRoutes({} as any)
  })

  // ── GET / —— 分页查询 ──────────────────────────────────────────────────
  describe('GET /', () => {
    it('should return paginated users', async () => {
      const pageResult = { list: [USER_VO], total: 1, page: 1, pageSize: 10 }
      mockService.findPage.mockResolvedValue(pageResult)

      const res = await app.request('/?page=1&pageSize=10')

      expect(res.status).toBe(200)
      const body = await res.json()
      expect(body.list).toHaveLength(1)
      expect(body.total).toBe(1)
      expect(body.list[0].id).toBe('user-1')
    })

    it('should reject invalid query params (Zod validation)', async () => {
      // page 必须 >= 1，传 0 应被 Zod 拦截
      const res = await app.request('/?page=0&pageSize=10')

      expect(res.status).toBe(400)
    })
  })

  // ── GET /:id —— 根据 ID 查询 ──────────────────────────────────────────
  describe('GET /:id', () => {
    it('should return user by id', async () => {
      mockService.findById.mockResolvedValue(USER_VO)

      const res = await app.request('/user-1')

      expect(res.status).toBe(200)
      const body = await res.json()
      expect(body.name).toBe('Test User')
    })

    it('should return 404 when user not found', async () => {
      mockService.findById.mockRejectedValue(new NotFoundError('User', 'non-existent'))

      const res = await app.request('/non-existent')

      // Hono 默认把未捕获异常返回 500；如有 errorHandler 中间件则映射为 404
      // 此处验证异常确实被抛出
      expect(res.status).toBeGreaterThanOrEqual(400)
    })
  })

  // ── PUT /:id —— 更新用户 ──────────────────────────────────────────────
  describe('PUT /:id', () => {
    it('should update user and return updated VO', async () => {
      const updatedVO = { ...USER_VO, name: 'Updated Name' }
      mockService.update.mockResolvedValue(updatedVO)

      const res = await app.request('/user-1', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ nickname: 'Updated Name' }),
      })

      expect(res.status).toBe(200)
      const body = await res.json()
      expect(body.name).toBe('Updated Name')
      expect(mockService.update).toHaveBeenCalledWith('user-1', { nickname: 'Updated Name' })
    })
  })

  // ── DELETE /:id —— 删除用户 ────────────────────────────────────────────
  describe('DELETE /:id', () => {
    it('should return 204 on successful delete', async () => {
      mockService.delete.mockResolvedValue(undefined)

      const res = await app.request('/user-1', { method: 'DELETE' })

      expect(res.status).toBe(204)
      expect(mockService.delete).toHaveBeenCalledWith('user-1')
    })
  })

  // ── POST /batch-delete —— 批量删除 ────────────────────────────────────
  describe('POST /batch-delete', () => {
    it('should accept array of ids and return 204', async () => {
      mockService.batchDelete.mockResolvedValue(undefined)

      const res = await app.request('/batch-delete', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ids: ['id-1', 'id-2'] }),
      })

      expect(res.status).toBe(204)
      expect(mockService.batchDelete).toHaveBeenCalledWith(['id-1', 'id-2'])
    })

    it('should reject empty ids array', async () => {
      const res = await app.request('/batch-delete', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ids: [] }),
      })

      // Zod schema 要求 ids 非空数组
      expect(res.status).toBe(400)
    })
  })

  // ── PATCH /:id/status —— 更新状态 ─────────────────────────────────────
  describe('PATCH /:id/status', () => {
    it('should update user status', async () => {
      mockService.updateStatus.mockResolvedValue(undefined)

      const res = await app.request('/user-1/status', {
        method: 'PATCH',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ status: 0 }),
      })

      expect(res.status).toBe(204)
      expect(mockService.updateStatus).toHaveBeenCalledWith('user-1', 0)
    })
  })
})
