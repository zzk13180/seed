import { Test } from '@nestjs/testing'
import { getRepositoryToken } from '@nestjs/typeorm'
import { NotFoundException, ConflictException, BadRequestException } from '@nestjs/common'
import * as bcrypt from 'bcrypt'
import { UserStatus } from '../../common/enums/user.enum'
import { UserService } from './user.service'
import { User } from './entities/user.entity'
import type { Repository } from 'typeorm'
import type { TestingModule } from '@nestjs/testing'

// Mock bcrypt
jest.mock('bcrypt', () => ({
  hash: jest.fn().mockResolvedValue('hashedPassword'),
  compare: jest.fn().mockResolvedValue(true),
}))

describe('UserService', () => {
  let service: UserService
  let repository: jest.Mocked<Repository<User>>

  const mockUser = {
    id: 1,
    username: 'testuser',
    password: 'hashedPassword',
    nickname: 'Test User',
    email: 'test@example.com',
    phone: '13800138000',
    avatar: null,
    status: UserStatus.ENABLED,
    deleted: false,
    createdAt: new Date(),
    updatedAt: new Date(),
    createdBy: null,
    updatedBy: null,
  } as User

  const mockRepository = {
    findOne: jest.fn(),
    find: jest.fn(),
    findAndCount: jest.fn(),
    create: jest.fn(),
    save: jest.fn(),
    update: jest.fn(),
  }

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        UserService,
        {
          provide: getRepositoryToken(User),
          useValue: mockRepository,
        },
      ],
    }).compile()

    service = module.get<UserService>(UserService)
    repository = module.get(getRepositoryToken(User))

    // Reset all mocks before each test
    jest.clearAllMocks()
  })

  it('should be defined', () => {
    expect(service).toBeDefined()
  })

  describe('findById', () => {
    it('should return a user when found', async () => {
      mockRepository.findOne.mockResolvedValue(mockUser)

      const result = await service.findById(1)

      expect(result).toEqual(mockUser)
      expect(mockRepository.findOne).toHaveBeenCalledWith({
        where: { id: 1, deleted: false },
      })
    })

    it('should throw NotFoundException when user not found', async () => {
      mockRepository.findOne.mockResolvedValue(null)

      await expect(service.findById(999)).rejects.toThrow(NotFoundException)
    })
  })

  describe('findByUsername', () => {
    it('should return a user when found', async () => {
      mockRepository.findOne.mockResolvedValue(mockUser)

      const result = await service.findByUsername('testuser')

      expect(result).toEqual(mockUser)
      expect(mockRepository.findOne).toHaveBeenCalledWith({
        where: { username: 'testuser', deleted: false },
      })
    })

    it('should return null when user not found', async () => {
      mockRepository.findOne.mockResolvedValue(null)

      const result = await service.findByUsername('nonexistent')

      expect(result).toBeNull()
    })
  })

  describe('findAll', () => {
    it('should return all users', async () => {
      const users = [mockUser, { ...mockUser, id: 2, username: 'testuser2' }]
      mockRepository.find.mockResolvedValue(users)

      const result = await service.findAll()

      expect(result).toEqual(users)
      expect(mockRepository.find).toHaveBeenCalledWith({
        where: { deleted: false },
        order: { createdAt: 'DESC' },
      })
    })
  })

  describe('page', () => {
    it('should return paginated users', async () => {
      const users = [mockUser]
      mockRepository.findAndCount.mockResolvedValue([users, 1])

      const result = await service.page({
        page: 1,
        pageSize: 10,
        getSkip: () => 0,
        getTake: () => 10,
      } as any)

      expect(result.list).toEqual(users)
      expect(result.total).toBe(1)
      expect(result.page).toBe(1)
      expect(result.pageSize).toBe(10)
    })

    it('should filter by username', async () => {
      mockRepository.findAndCount.mockResolvedValue([[], 0])

      await service.page({
        page: 1,
        pageSize: 10,
        username: 'test',
        getSkip: () => 0,
        getTake: () => 10,
      } as any)

      expect(mockRepository.findAndCount).toHaveBeenCalledWith(
        expect.objectContaining({
          where: expect.objectContaining({
            deleted: false,
          }),
        }),
      )
    })
  })

  describe('create', () => {
    const createDto = {
      username: 'newuser',
      password: 'password123',
      nickname: 'New User',
      email: 'new@example.com',
    }

    it('should create a new user', async () => {
      mockRepository.findOne.mockResolvedValue(null) // No existing user
      mockRepository.create.mockReturnValue({ ...mockUser, ...createDto })
      mockRepository.save.mockResolvedValue({ ...mockUser, ...createDto })

      const result = await service.create(createDto)

      expect(result.username).toBe(createDto.username)
      expect(bcrypt.hash).toHaveBeenCalledWith(createDto.password, 10)
    })

    it('should throw ConflictException when username exists', async () => {
      mockRepository.findOne.mockResolvedValue(mockUser)

      await expect(service.create(createDto)).rejects.toThrow(ConflictException)
    })

    it('should throw ConflictException when email exists', async () => {
      mockRepository.findOne
        .mockResolvedValueOnce(null) // No user with same username
        .mockResolvedValueOnce(mockUser) // User with same email exists

      await expect(service.create(createDto)).rejects.toThrow(ConflictException)
    })
  })

  describe('update', () => {
    const updateDto = {
      nickname: 'Updated User',
      email: 'updated@example.com',
    }

    it('should update a user', async () => {
      mockRepository.findOne
        .mockResolvedValueOnce(mockUser) // findById
        .mockResolvedValueOnce(null) // Check email uniqueness
      mockRepository.save.mockResolvedValue({ ...mockUser, ...updateDto })

      const result = await service.update(1, updateDto)

      expect(result.nickname).toBe(updateDto.nickname)
    })

    it('should throw NotFoundException when user not found', async () => {
      mockRepository.findOne.mockResolvedValue(null)

      await expect(service.update(999, updateDto)).rejects.toThrow(NotFoundException)
    })
  })

  describe('delete', () => {
    it('should soft delete a user', async () => {
      mockRepository.findOne.mockResolvedValue(mockUser)
      mockRepository.save.mockResolvedValue({ ...mockUser, deleted: true })

      await service.delete(1)

      expect(mockRepository.save).toHaveBeenCalledWith(expect.objectContaining({ deleted: true }))
    })

    it('should throw NotFoundException when user not found', async () => {
      mockRepository.findOne.mockResolvedValue(null)

      await expect(service.delete(999)).rejects.toThrow(NotFoundException)
    })
  })

  describe('deleteBatch', () => {
    it('should soft delete multiple users', async () => {
      mockRepository.update.mockResolvedValue({ affected: 2 } as any)

      await service.deleteBatch([1, 2])

      expect(mockRepository.update).toHaveBeenCalledWith([1, 2], { deleted: true })
    })
  })

  describe('updateStatus', () => {
    it('should update user status', async () => {
      mockRepository.findOne.mockResolvedValue(mockUser)
      mockRepository.save.mockResolvedValue({ ...mockUser, status: UserStatus.DISABLED })

      await service.updateStatus(1, UserStatus.DISABLED)

      expect(mockRepository.save).toHaveBeenCalledWith(
        expect.objectContaining({ status: UserStatus.DISABLED }),
      )
    })
  })

  describe('resetPassword', () => {
    it('should reset user password', async () => {
      mockRepository.findOne.mockResolvedValue(mockUser)
      mockRepository.save.mockResolvedValue(mockUser)

      await service.resetPassword(1, 'newpassword123')

      expect(bcrypt.hash).toHaveBeenCalledWith('newpassword123', 10)
      expect(mockRepository.save).toHaveBeenCalled()
    })

    it('should throw BadRequestException when password is too short', async () => {
      mockRepository.findOne.mockResolvedValue(mockUser)

      await expect(service.resetPassword(1, '12345')).rejects.toThrow(BadRequestException)
    })
  })

  describe('validatePassword', () => {
    it('should return true for valid password', async () => {
      ;(bcrypt.compare as jest.Mock).mockResolvedValue(true)

      const result = await service.validatePassword(mockUser, 'password123')

      expect(result).toBe(true)
      expect(bcrypt.compare).toHaveBeenCalledWith('password123', mockUser.password)
    })

    it('should return false for invalid password', async () => {
      ;(bcrypt.compare as jest.Mock).mockResolvedValue(false)

      const result = await service.validatePassword(mockUser, 'wrongpassword')

      expect(result).toBe(false)
    })
  })
})
