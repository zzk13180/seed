import { Test } from '@nestjs/testing'
import { JwtService } from '@nestjs/jwt'
import { ConfigService } from '@nestjs/config'
import { CACHE_MANAGER } from '@nestjs/cache-manager'
import { UnauthorizedException } from '@nestjs/common'
import { UserService } from '../user/user.service'
import { UserStatus } from '../../common/enums/user.enum'
import { AuthService } from './auth.service'
import type { TestingModule } from '@nestjs/testing'

describe('AuthService', () => {
  let service: AuthService
  let userService: jest.Mocked<UserService>
  let jwtService: jest.Mocked<JwtService>
  let cacheManager: jest.Mocked<any>

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
  }

  const mockUserService = {
    findByUsername: jest.fn(),
    findById: jest.fn(),
    validatePassword: jest.fn(),
  }

  const mockJwtService = {
    sign: jest.fn().mockReturnValue('mock-jwt-token'),
    verify: jest.fn(),
  }

  const mockConfigService = {
    get: jest.fn().mockReturnValue('7d'),
  }

  const mockCacheManager = {
    get: jest.fn(),
    set: jest.fn(),
    del: jest.fn(),
  }

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        AuthService,
        {
          provide: UserService,
          useValue: mockUserService,
        },
        {
          provide: JwtService,
          useValue: mockJwtService,
        },
        {
          provide: ConfigService,
          useValue: mockConfigService,
        },
        {
          provide: CACHE_MANAGER,
          useValue: mockCacheManager,
        },
      ],
    }).compile()

    service = module.get<AuthService>(AuthService)
    userService = module.get(UserService)
    jwtService = module.get(JwtService)
    cacheManager = module.get(CACHE_MANAGER)

    jest.clearAllMocks()
  })

  it('should be defined', () => {
    expect(service).toBeDefined()
  })

  describe('login', () => {
    const loginDto = { username: 'testuser', password: 'password123' }

    it('should return login response with tokens', async () => {
      mockCacheManager.get.mockResolvedValue(null) // No login attempts
      mockUserService.findByUsername.mockResolvedValue(mockUser)
      mockUserService.validatePassword.mockResolvedValue(true)

      const result = await service.login(loginDto)

      expect(result).toHaveProperty('accessToken')
      expect(result).toHaveProperty('refreshToken')
      expect(result).toHaveProperty('user')
      expect(result).toHaveProperty('expiresIn')
      expect(result.accessToken).toBe('mock-jwt-token')
    })

    it('should throw UnauthorizedException when user not found', async () => {
      mockCacheManager.get.mockResolvedValue(null)
      mockUserService.findByUsername.mockResolvedValue(null)

      await expect(service.login(loginDto)).rejects.toThrow(UnauthorizedException)
    })

    it('should throw UnauthorizedException when account is disabled', async () => {
      mockCacheManager.get.mockResolvedValue(null)
      mockUserService.findByUsername.mockResolvedValue({
        ...mockUser,
        status: UserStatus.DISABLED,
      })

      await expect(service.login(loginDto)).rejects.toThrow(UnauthorizedException)
    })

    it('should throw UnauthorizedException when password is invalid', async () => {
      mockCacheManager.get.mockResolvedValue(null)
      mockUserService.findByUsername.mockResolvedValue(mockUser)
      mockUserService.validatePassword.mockResolvedValue(false)

      await expect(service.login(loginDto)).rejects.toThrow(UnauthorizedException)
    })

    it('should throw UnauthorizedException when too many login attempts', async () => {
      mockCacheManager.get.mockResolvedValue(5) // Max attempts reached

      await expect(service.login(loginDto)).rejects.toThrow(UnauthorizedException)
    })

    it('should increment login attempts on failed login', async () => {
      mockCacheManager.get.mockResolvedValue(null)
      mockUserService.findByUsername.mockResolvedValue(null)

      await expect(service.login(loginDto)).rejects.toThrow(UnauthorizedException)
      expect(mockCacheManager.set).toHaveBeenCalled()
    })

    it('should clear login attempts on successful login', async () => {
      mockCacheManager.get.mockResolvedValue(2)
      mockUserService.findByUsername.mockResolvedValue(mockUser)
      mockUserService.validatePassword.mockResolvedValue(true)

      await service.login(loginDto)

      expect(mockCacheManager.del).toHaveBeenCalledWith('login_attempts:testuser')
    })
  })

  describe('refreshToken', () => {
    it('should return new tokens with valid refresh token', async () => {
      mockJwtService.verify.mockReturnValue({
        sub: 1,
        username: 'testuser',
        roles: ['USER'],
        permissions: ['user:read'],
      })
      mockUserService.findById.mockResolvedValue(mockUser)

      const result = await service.refreshToken('valid-refresh-token')

      expect(result).toHaveProperty('accessToken')
      expect(result).toHaveProperty('refreshToken')
      expect(mockJwtService.verify).toHaveBeenCalledWith('valid-refresh-token')
    })

    it('should throw UnauthorizedException with invalid refresh token', async () => {
      mockJwtService.verify.mockImplementation(() => {
        throw new Error('Invalid token')
      })

      await expect(service.refreshToken('invalid-token')).rejects.toThrow(UnauthorizedException)
    })
  })

  describe('logout', () => {
    it('should add user to blacklist', async () => {
      const currentUser = { id: 1, username: 'testuser' }

      await service.logout(currentUser)

      expect(mockCacheManager.set).toHaveBeenCalledWith(
        'blacklist:1',
        'logged_out',
        expect.any(Number),
      )
    })
  })

  describe('getCurrentUser', () => {
    it('should return current user info', async () => {
      const currentUser = { id: 1, username: 'testuser' }
      mockUserService.findById.mockResolvedValue(mockUser)

      const result = await service.getCurrentUser(currentUser)

      expect(result).toHaveProperty('user')
      expect(mockUserService.findById).toHaveBeenCalledWith(1)
    })
  })
})
