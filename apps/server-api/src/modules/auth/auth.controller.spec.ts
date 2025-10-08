import { Test } from '@nestjs/testing'
import { ResponseDto } from '../../common/dto/response.dto'
import { AuthController } from './auth.controller'
import { AuthService } from './auth.service'
import type { TestingModule } from '@nestjs/testing'

describe('AuthController', () => {
  let controller: AuthController
  let service: jest.Mocked<AuthService>

  const mockLoginResponse = {
    accessToken: 'mock-access-token',
    refreshToken: 'mock-refresh-token',
    user: {
      id: 1,
      username: 'testuser',
      nickname: 'Test User',
      email: 'test@example.com',
    },
    expiresIn: 3600,
  }

  const mockAuthService = {
    login: jest.fn(),
    refreshToken: jest.fn(),
    logout: jest.fn(),
    getCurrentUser: jest.fn(),
  }

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      controllers: [AuthController],
      providers: [
        {
          provide: AuthService,
          useValue: mockAuthService,
        },
      ],
    }).compile()

    controller = module.get<AuthController>(AuthController)
    service = module.get(AuthService)

    jest.clearAllMocks()
  })

  it('should be defined', () => {
    expect(controller).toBeDefined()
  })

  describe('login', () => {
    it('should return login response with tokens', async () => {
      const loginDto = { username: 'testuser', password: 'password123' }
      mockAuthService.login.mockResolvedValue(mockLoginResponse)

      const result = await controller.login(loginDto)

      expect(result).toBeInstanceOf(ResponseDto)
      expect(result.code).toBe(200)
      expect(result.data).toEqual(mockLoginResponse)
      expect(mockAuthService.login).toHaveBeenCalledWith(loginDto)
    })
  })

  describe('refresh', () => {
    it('should return new tokens', async () => {
      mockAuthService.refreshToken.mockResolvedValue(mockLoginResponse)

      const result = await controller.refresh('refresh-token')

      expect(result).toBeInstanceOf(ResponseDto)
      expect(result.code).toBe(200)
      expect(result.data).toEqual(mockLoginResponse)
      expect(mockAuthService.refreshToken).toHaveBeenCalledWith('refresh-token')
    })
  })

  describe('logout', () => {
    it('should call logout service', async () => {
      const currentUser = { id: 1, username: 'testuser' }
      mockAuthService.logout.mockResolvedValue(undefined)

      await controller.logout(currentUser)

      expect(mockAuthService.logout).toHaveBeenCalledWith(currentUser)
    })
  })

  describe('me', () => {
    it('should return current user info', async () => {
      const currentUser = { id: 1, username: 'testuser' }
      mockAuthService.getCurrentUser.mockResolvedValue(mockLoginResponse)

      const result = await controller.me(currentUser)

      expect(result).toBeInstanceOf(ResponseDto)
      expect(result.code).toBe(200)
      expect(mockAuthService.getCurrentUser).toHaveBeenCalledWith(currentUser)
    })
  })
})
