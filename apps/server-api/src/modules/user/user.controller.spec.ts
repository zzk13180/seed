import { Test } from '@nestjs/testing'
import { UserStatus } from '../../common/enums/user.enum'
import { ResponseDto } from '../../common/dto/response.dto'
import { PageResultDto } from '../../common/dto/page-result.dto'
import { UserService } from './user.service'
import { UserController } from './user.controller'
import type { TestingModule } from '@nestjs/testing'

describe('UserController', () => {
  let controller: UserController
  let service: jest.Mocked<UserService>

  const mockUser = {
    id: 1,
    username: 'testuser',
    nickname: 'Test User',
    email: 'test@example.com',
    phone: '13800138000',
    avatar: null,
    status: UserStatus.ENABLED,
    createdAt: new Date(),
    updatedAt: new Date(),
  }

  const mockUserService = {
    page: jest.fn(),
    findAll: jest.fn(),
    findById: jest.fn(),
    create: jest.fn(),
    update: jest.fn(),
    delete: jest.fn(),
    deleteBatch: jest.fn(),
    updateStatus: jest.fn(),
    resetPassword: jest.fn(),
  }

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      controllers: [UserController],
      providers: [
        {
          provide: UserService,
          useValue: mockUserService,
        },
      ],
    }).compile()

    controller = module.get<UserController>(UserController)
    service = module.get(UserService)

    jest.clearAllMocks()
  })

  it('should be defined', () => {
    expect(controller).toBeDefined()
  })

  describe('page', () => {
    it('should return paginated users', async () => {
      const pageResult = PageResultDto.create([mockUser], 1, 1, 10)
      mockUserService.page.mockResolvedValue(pageResult)

      const result = await controller.page({ page: 1, pageSize: 10 } as any)

      expect(result).toBeInstanceOf(ResponseDto)
      expect(result.code).toBe(200)
      expect(result.data).toEqual(pageResult)
    })
  })

  describe('list', () => {
    it('should return all users', async () => {
      mockUserService.findAll.mockResolvedValue([mockUser])

      const result = await controller.list()

      expect(result).toBeInstanceOf(ResponseDto)
      expect(result.code).toBe(200)
      expect(result.data).toEqual([mockUser])
    })
  })

  describe('getById', () => {
    it('should return a user by id', async () => {
      mockUserService.findById.mockResolvedValue(mockUser)

      const result = await controller.getById(1)

      expect(result).toBeInstanceOf(ResponseDto)
      expect(result.code).toBe(200)
      expect(result.data).toEqual(mockUser)
    })
  })

  describe('create', () => {
    it('should create a new user', async () => {
      const createDto = {
        username: 'newuser',
        password: 'password123',
        nickname: 'New User',
      }
      mockUserService.create.mockResolvedValue({ ...mockUser, ...createDto })

      const result = await controller.create(createDto)

      expect(result).toBeInstanceOf(ResponseDto)
      expect(result.code).toBe(200)
      expect(mockUserService.create).toHaveBeenCalledWith(createDto)
    })
  })

  describe('update', () => {
    it('should update a user', async () => {
      const updateDto = { nickname: 'Updated User' }
      mockUserService.update.mockResolvedValue({ ...mockUser, ...updateDto })

      const result = await controller.update(1, updateDto)

      expect(result).toBeInstanceOf(ResponseDto)
      expect(result.code).toBe(200)
      expect(mockUserService.update).toHaveBeenCalledWith(1, updateDto)
    })
  })

  describe('delete', () => {
    it('should delete a user', async () => {
      mockUserService.delete.mockResolvedValue(undefined)

      await controller.delete(1)

      expect(mockUserService.delete).toHaveBeenCalledWith(1)
    })
  })

  describe('deleteBatch', () => {
    it('should delete multiple users', async () => {
      mockUserService.deleteBatch.mockResolvedValue(undefined)

      await controller.deleteBatch([1, 2, 3])

      expect(mockUserService.deleteBatch).toHaveBeenCalledWith([1, 2, 3])
    })
  })

  describe('updateStatus', () => {
    it('should update user status', async () => {
      mockUserService.updateStatus.mockResolvedValue(undefined)

      await controller.updateStatus(1, UserStatus.DISABLED)

      expect(mockUserService.updateStatus).toHaveBeenCalledWith(1, UserStatus.DISABLED)
    })
  })

  describe('resetPassword', () => {
    it('should reset user password', async () => {
      mockUserService.resetPassword.mockResolvedValue(undefined)

      await controller.resetPassword(1, 'newpassword123')

      expect(mockUserService.resetPassword).toHaveBeenCalledWith(1, 'newpassword123')
    })
  })
})
