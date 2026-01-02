import { vi, describe, it, expect, beforeEach, type Mock, type Mocked } from 'vitest'
import { Test } from '@nestjs/testing'
import * as bcrypt from 'bcrypt'
import { UserException } from '../../common/exceptions/business.exception'
import { UserStatus } from '../../common/enums/user.enum'
import { DRIZZLE_DB } from '../../common/database/drizzle.constants'
import { UserService } from './user.service'
import { UserMapper } from './user.mapper'
import type { TestingModule } from '@nestjs/testing'

// Mock bcrypt
vi.mock('bcrypt', () => ({
  hash: vi.fn().mockResolvedValue('hashedPassword'),
  compare: vi.fn().mockResolvedValue(true),
}))

describe('UserService', () => {
  let service: UserService

  const mockUser = {
    id: 1,
    username: 'testuser',
    password: 'hashedPassword',
    nickname: 'Test User',
    email: 'test@example.com',
    phone: '13800138000',
    avatar: null,
    status: UserStatus.ENABLED,
    deleted: 0,
    createdAt: new Date(),
    updatedAt: new Date(),
    createdBy: null,
    updatedBy: null,
  }

  // Mock Drizzle DB with chainable methods
  const createMockDb = () => {
    const mockResult = {
      select: vi.fn().mockReturnThis(),
      from: vi.fn().mockReturnThis(),
      where: vi.fn().mockReturnThis(),
      orderBy: vi.fn().mockReturnThis(),
      offset: vi.fn().mockReturnThis(),
      limit: vi.fn().mockReturnThis(),
      insert: vi.fn().mockReturnThis(),
      values: vi.fn().mockResolvedValue([{ insertId: BigInt(1) }]),
      update: vi.fn().mockReturnThis(),
      set: vi.fn().mockReturnThis(),
      execute: vi.fn().mockResolvedValue([]),
    }

    // Make select() return a promise that resolves to an array when awaited
    let selectResult: any[] = []
    let countResult = [{ count: 0 }]

    const db = {
      select: vi.fn().mockImplementation((cols?: any) => {
        if (cols?.count) {
          return {
            from: vi.fn().mockReturnValue({
              where: vi.fn().mockResolvedValue(countResult),
            }),
          }
        }
        return {
          from: vi.fn().mockReturnValue({
            where: vi.fn().mockReturnValue({
              orderBy: vi.fn().mockReturnValue({
                offset: vi.fn().mockReturnValue({
                  limit: vi.fn().mockResolvedValue(selectResult),
                }),
              }),
              limit: vi.fn().mockResolvedValue(selectResult),
            }),
            orderBy: vi.fn().mockResolvedValue(selectResult),
          }),
        }
      }),
      insert: vi.fn().mockReturnValue({
        values: vi.fn().mockResolvedValue([{ insertId: BigInt(1) }]),
      }),
      update: vi.fn().mockReturnValue({
        set: vi.fn().mockReturnValue({
          where: vi.fn().mockResolvedValue([]),
        }),
      }),
      _setSelectResult: (result: any[]) => {
        selectResult = result
      },
      _setCountResult: (count: number) => {
        countResult = [{ count }]
      },
    }

    return db
  }

  let mockDb: ReturnType<typeof createMockDb>

  const mockUserMapper = {
    toVO: vi.fn((user: unknown) => user),
    toVOList: vi.fn((users: unknown) => users),
    toLoginUserVO: vi.fn((user: unknown) => user),
  }

  beforeEach(async () => {
    mockDb = createMockDb()

    const module: TestingModule = await Test.createTestingModule({
      providers: [
        UserService,
        {
          provide: DRIZZLE_DB,
          useValue: mockDb,
        },
        {
          provide: UserMapper,
          useValue: mockUserMapper,
        },
      ],
    }).compile()

    service = module.get<UserService>(UserService)

    vi.clearAllMocks()
  })

  it('should be defined', () => {
    expect(service).toBeDefined()
  })

  describe('validatePassword', () => {
    it('should return true for valid password', async () => {
      ;(bcrypt.compare as Mock).mockResolvedValue(true)

      const result = await service.validatePassword(mockUser as any, 'password123')

      expect(result).toBe(true)
      expect(bcrypt.compare).toHaveBeenCalledWith('password123', mockUser.password)
    })

    it('should return false for invalid password', async () => {
      ;(bcrypt.compare as Mock).mockResolvedValue(false)

      const result = await service.validatePassword(mockUser as any, 'wrongpassword')

      expect(result).toBe(false)
    })
  })
})
