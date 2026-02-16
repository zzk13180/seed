/**
 * UserService 单元测试 — 后端 Service 层测试示例
 *
 * 测试策略：
 * - Mock UserRepository（数据访问层），隔离测试业务逻辑
 * - 注入 mock hashPassword 函数，避免依赖 Bun 运行时
 * - 每个测试独立构造数据，不共享可变状态
 *
 * 覆盖模式：
 * - 正常路径 (happy path)
 * - 边界条件 (not found / conflict)
 * - 字段映射 (nickname→name, avatar→image)
 */
import { describe, it, expect, vi, beforeEach } from 'vitest'
import { NotFoundError, ConflictError } from '@seed/contracts/errors'
import { UserService } from '../user.service.js'
import type { User } from '@seed/db/schema'
import type { UserQuery } from '@seed/contracts/schemas/user'

// 必须在 vi.mock 之后 import（此时 UserService 内部的 new UserRepository() 会得到 mockRepo）

// ─── Mock Setup ──────────────────────────────────────────────────────────────
// vi.hoisted 让变量在 vi.mock 工厂中可用（vitest 会把 vi.mock 提升到文件顶部）
const mockRepo = vi.hoisted(() => ({
  findPage: vi.fn(),
  findAll: vi.fn(),
  findById: vi.fn(),
  findByEmail: vi.fn(),
  update: vi.fn(),
  softDelete: vi.fn(),
  batchSoftDelete: vi.fn(),
  updateStatus: vi.fn(),
  resetPassword: vi.fn(),
}))

vi.mock('../user.repository.js', () => ({
  UserRepository: vi.fn(() => mockRepo),
}))

// ─── Test Fixtures ───────────────────────────────────────────────────────────
const NOW = new Date('2025-01-15T10:00:00Z')

/** 构造 User entity stub（模拟 DB 返回的完整行） */
function createUserEntity(overrides: Partial<User> = {}): User {
  return {
    id: 'user-1',
    name: 'Test User',
    email: 'test@example.com',
    emailVerified: true,
    image: null,
    phone: null,
    role: 'user',
    status: 1,
    deleted: 0,
    createdAt: NOW,
    updatedAt: NOW,
    ...overrides,
  }
}

/** mock 密码哈希——返回固定前缀便于断言 */
const mockHashPassword = vi.fn(async (pw: string) => `hashed:${pw}`)

// ─── Tests ───────────────────────────────────────────────────────────────────
describe('UserService', () => {
  let service: UserService

  beforeEach(() => {
    vi.clearAllMocks()
    // db 参数被 mock 的 UserRepository 忽略，传 {} 即可
    service = new UserService({} as any, mockHashPassword)
  })

  // ── findPage ─────────────────────────────────────────────────────────────
  describe('findPage', () => {
    it('should return paginated UserVO list', async () => {
      const entities = [createUserEntity(), createUserEntity({ id: 'user-2', name: 'User 2' })]
      mockRepo.findPage.mockResolvedValue({ list: entities, total: 2 })

      const query: UserQuery = { page: 1, pageSize: 10, orderDirection: 'DESC' }
      const result = await service.findPage(query)

      expect(result.list).toHaveLength(2)
      expect(result.total).toBe(2)
      expect(result.page).toBe(1)
      expect(result.pageSize).toBe(10)
      // VO 转换：Date → ISO string
      expect(result.list[0]!.createdAt).toBe(NOW.toISOString())
      expect(mockRepo.findPage).toHaveBeenCalledWith(query)
    })
  })

  // ── findById ─────────────────────────────────────────────────────────────
  describe('findById', () => {
    it('should return UserVO when user exists', async () => {
      const entity = createUserEntity()
      mockRepo.findById.mockResolvedValue(entity)

      const vo = await service.findById('user-1')

      expect(vo.id).toBe('user-1')
      expect(vo.name).toBe('Test User')
      expect(vo.email).toBe('test@example.com')
      // 确认不暴露内部字段
      expect(vo).not.toHaveProperty('deleted')
    })

    it('should throw NotFoundError when user does not exist', async () => {
      mockRepo.findById.mockResolvedValue(undefined)

      await expect(service.findById('non-existent')).rejects.toThrow(NotFoundError)
    })
  })

  // ── update ───────────────────────────────────────────────────────────────
  describe('update', () => {
    it('should update and return UserVO with field mapping', async () => {
      const entity = createUserEntity()
      const updatedEntity = createUserEntity({ name: 'New Name', image: 'https://img.png' })

      mockRepo.findById.mockResolvedValue(entity)
      mockRepo.update.mockResolvedValue(updatedEntity)

      const vo = await service.update('user-1', {
        nickname: 'New Name',
        avatar: 'https://img.png',
      })

      // 验证字段映射：nickname→name, avatar→image
      expect(mockRepo.update).toHaveBeenCalledWith('user-1', {
        name: 'New Name',
        image: 'https://img.png',
      })
      expect(vo.name).toBe('New Name')
      expect(vo.image).toBe('https://img.png')
    })

    it('should throw NotFoundError when user does not exist', async () => {
      mockRepo.findById.mockResolvedValue(undefined)

      await expect(service.update('non-existent', { nickname: 'x' })).rejects.toThrow(NotFoundError)
    })

    it('should throw ConflictError when email already taken', async () => {
      const entity = createUserEntity()
      const conflictUser = createUserEntity({ id: 'user-other', email: 'taken@example.com' })

      mockRepo.findById.mockResolvedValue(entity)
      mockRepo.findByEmail.mockResolvedValue(conflictUser)

      await expect(service.update('user-1', { email: 'taken@example.com' })).rejects.toThrow(
        ConflictError,
      )
    })

    it('should skip email uniqueness check when email unchanged', async () => {
      const entity = createUserEntity({ email: 'same@example.com' })
      mockRepo.findById.mockResolvedValue(entity)
      mockRepo.update.mockResolvedValue(entity)

      await service.update('user-1', { email: 'same@example.com' })

      // findByEmail 不应被调用
      expect(mockRepo.findByEmail).not.toHaveBeenCalled()
    })
  })

  // ── delete ───────────────────────────────────────────────────────────────
  describe('delete', () => {
    it('should soft-delete existing user', async () => {
      mockRepo.findById.mockResolvedValue(createUserEntity())

      await service.delete('user-1')

      expect(mockRepo.softDelete).toHaveBeenCalledWith('user-1')
    })

    it('should throw NotFoundError when user does not exist', async () => {
      mockRepo.findById.mockResolvedValue(undefined)

      await expect(service.delete('non-existent')).rejects.toThrow(NotFoundError)
    })
  })

  // ── resetPassword ────────────────────────────────────────────────────────
  describe('resetPassword', () => {
    it('should hash password before saving', async () => {
      mockRepo.findById.mockResolvedValue(createUserEntity())

      await service.resetPassword('user-1', 'newPass123')

      // 验证密码经过 hash 再传给 repository
      expect(mockHashPassword).toHaveBeenCalledWith('newPass123')
      expect(mockRepo.resetPassword).toHaveBeenCalledWith('user-1', 'hashed:newPass123')
    })
  })

  // ── batchDelete ──────────────────────────────────────────────────────────
  describe('batchDelete', () => {
    it('should delegate to repository without existence check', async () => {
      await service.batchDelete(['id-1', 'id-2', 'id-3'])

      expect(mockRepo.batchSoftDelete).toHaveBeenCalledWith(['id-1', 'id-2', 'id-3'])
      // batchDelete 不做逐个存在性检查
      expect(mockRepo.findById).not.toHaveBeenCalled()
    })
  })
})
