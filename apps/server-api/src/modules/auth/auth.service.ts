import { randomUUID } from 'node:crypto'
import { Injectable, Inject, Logger } from '@nestjs/common'
import { JwtService } from '@nestjs/jwt'
import { ConfigService } from '@nestjs/config'
import { CACHE_MANAGER } from '@nestjs/cache-manager'
import { UserService } from '../user/user.service'
import { UserMapper } from '../user/user.mapper'
import { UserVo } from '../user/vo/user.vo'
import { UserStatus } from '../../common/enums/user.enum'
import { UnauthorizedException as BizUnauthorizedException } from '../../common/exceptions/business.exception'
import { LoginDto } from './dto/login.dto'
import { LoginVo } from './vo/login.vo'
import type { JwtPayload, CurrentUser } from '../../common/interfaces/auth.interface'
import type { Cache } from 'cache-manager'

/**
 * 认证服务
 */
@Injectable()
export class AuthService {
  private readonly logger = new Logger(AuthService.name)
  private readonly MAX_LOGIN_ATTEMPTS = 5
  private readonly LOCKOUT_DURATION = 15 * 60 // 15 分钟（秒）
  private readonly LOCKOUT_MINUTES = 15

  constructor(
    private readonly userService: UserService,
    private readonly userMapper: UserMapper,
    private readonly jwtService: JwtService,
    private readonly configService: ConfigService,
    @Inject(CACHE_MANAGER) private readonly cacheManager: Cache,
  ) {}

  /**
   * 用户登录
   */
  async login(dto: LoginDto): Promise<LoginVo> {
    // 检查账户是否被锁定
    await this.checkAccountLocked(dto.username)

    // 检查登录尝试次数
    await this.checkLoginAttempts(dto.username)

    // 验证用户
    const user = await this.userService.findByUsername(dto.username)
    if (!user) {
      await this.incrementLoginAttempts(dto.username)
      throw BizUnauthorizedException.invalidCredentials()
    }

    // 检查用户状态
    if (user.status === UserStatus.DISABLED) {
      throw BizUnauthorizedException.accountDisabled()
    }

    // 验证密码
    const isPasswordValid = await this.userService.validatePassword(user, dto.password)
    if (!isPasswordValid) {
      await this.incrementLoginAttempts(dto.username)
      throw BizUnauthorizedException.invalidCredentials()
    }

    // 清除登录尝试记录
    await this.clearLoginAttempts(dto.username)

    this.logger.log(`User ${dto.username} logged in successfully`)

    // TODO: 从数据库查询用户角色和权限，这里暂时硬编码
    // 实际项目中应该实现 RBAC 模型
    const roles = await this.getUserRoles(user.id)
    const permissions = await this.getUserPermissions(user.id)

    // 生成唯一的 JWT ID，用于黑名单功能
    const jti = randomUUID()

    // 生成令牌
    const payload: JwtPayload = {
      sub: user.id,
      username: user.username,
      roles,
      permissions,
      jti,
    }

    const accessToken = this.jwtService.sign(payload)
    const refreshToken = this.jwtService.sign(payload, {
      expiresIn: this.configService.get('jwt.refreshTokenExpiry'),
    })

    // 使用 mapper 构建用户信息（安全排除密码）
    const userVo = this.userMapper.toLoginUserVO(user)

    return {
      accessToken,
      refreshToken,
      user: userVo,
      expiresIn: 3600, // 1 小时
    }
  }

  /**
   * 刷新令牌
   */
  async refreshToken(refreshToken: string): Promise<LoginVo> {
    try {
      const payload = this.jwtService.verify(refreshToken)

      // 检查旧 token 是否已被拉黑
      if (payload.jti) {
        const isBlacklisted = await this.cacheManager.get(`blacklist:token:${payload.jti}`)
        if (isBlacklisted) {
          throw BizUnauthorizedException.tokenInvalid()
        }
      }

      // 获取 User Entity 用于生成新的 payload
      const user = await this.userService.findEntityById(payload.sub)

      // 重新查询用户的角色和权限，确保使用最新的权限数据
      const roles = await this.getUserRoles(user.id)
      const permissions = await this.getUserPermissions(user.id)

      // 生成新的 JWT ID
      const jti = randomUUID()

      const newPayload: JwtPayload = {
        sub: user.id,
        username: user.username,
        roles,
        permissions,
        jti,
      }

      const newAccessToken = this.jwtService.sign(newPayload)
      const newRefreshToken = this.jwtService.sign(newPayload, {
        expiresIn: this.configService.get('jwt.refreshTokenExpiry'),
      })

      // 使用 mapper 转换为 VO，确保不泄露敏感数据
      const userVo = this.userMapper.toLoginUserVO(user)

      return {
        accessToken: newAccessToken,
        refreshToken: newRefreshToken,
        user: userVo,
        expiresIn: 3600,
      }
    } catch {
      throw BizUnauthorizedException.tokenInvalid()
    }
  }

  /**
   * 用户登出
   */
  async logout(user: CurrentUser): Promise<void> {
    // 使用 jti 作为黑名单 key，支持多设备登录场景
    if (user.jti) {
      await this.cacheManager.set(
        `blacklist:token:${user.jti}`,
        'logged_out',
        60 * 60 * 24 * 1000, // 24 小时（毫秒）
      )
    }
    this.logger.log(`User ${user.username} logged out`)
  }

  /**
   * 获取当前用户信息
   */
  async getCurrentUser(user: CurrentUser): Promise<UserVo> {
    return this.userService.findById(user.id)
  }

  /**
   * 获取剩余登录尝试次数
   */
  async getRemainingAttempts(username: string): Promise<number> {
    const key = `login:attempts:${username}`
    const attempts = (await this.cacheManager.get<number>(key)) || 0
    return Math.max(0, this.MAX_LOGIN_ATTEMPTS - attempts)
  }

  /**
   * 检查账户是否被锁定
   */
  private async checkAccountLocked(username: string): Promise<void> {
    const lockKey = `login:lock:${username}`
    const isLocked = await this.cacheManager.get<boolean>(lockKey)

    if (isLocked) {
      this.logger.warn(`Account ${username} is locked due to too many failed attempts`)
      throw BizUnauthorizedException.tooManyAttempts(this.LOCKOUT_MINUTES)
    }
  }

  /**
   * 检查登录尝试次数
   */
  private async checkLoginAttempts(username: string): Promise<void> {
    const key = `login:attempts:${username}`
    const attempts = await this.cacheManager.get<number>(key)

    if (attempts && attempts >= this.MAX_LOGIN_ATTEMPTS) {
      // 锁定账户
      const lockKey = `login:lock:${username}`
      await this.cacheManager.set(lockKey, true, this.LOCKOUT_DURATION * 1000)

      this.logger.warn(`Account ${username} locked after ${attempts} failed attempts`)
      throw BizUnauthorizedException.tooManyAttempts(this.LOCKOUT_MINUTES)
    }
  }

  /**
   * 增加登录尝试次数
   */
  private async incrementLoginAttempts(username: string): Promise<void> {
    const key = `login:attempts:${username}`
    const attempts = (await this.cacheManager.get<number>(key)) || 0
    const newAttempts = attempts + 1

    // 设置过期时间为 1 小时
    await this.cacheManager.set(key, newAttempts, 3600 * 1000)

    this.logger.warn(`Login attempt ${newAttempts}/${this.MAX_LOGIN_ATTEMPTS} for user ${username}`)

    // 如果达到最大尝试次数，锁定账户
    if (newAttempts >= this.MAX_LOGIN_ATTEMPTS) {
      const lockKey = `login:lock:${username}`
      await this.cacheManager.set(lockKey, true, this.LOCKOUT_DURATION * 1000)
      this.logger.warn(`Account ${username} locked after ${newAttempts} failed attempts`)
    }
  }

  /**
   * 清除登录尝试记录
   */
  private async clearLoginAttempts(username: string): Promise<void> {
    const attemptsKey = `login:attempts:${username}`
    const lockKey = `login:lock:${username}`

    await this.cacheManager.del(attemptsKey)
    await this.cacheManager.del(lockKey)
  }

  /**
   * 获取用户角色
   * TODO: 实现 RBAC 模型后，从数据库查询用户角色
   */
  private async getUserRoles(_userId: number): Promise<string[]> {
    // 暂时返回默认角色，实际项目中应该从数据库查询
    // 例如：通过 user_roles 关联表查询
    return ['USER']
  }

  /**
   * 获取用户权限
   * TODO: 实现 RBAC 模型后，从数据库查询用户权限
   */
  private async getUserPermissions(_userId: number): Promise<string[]> {
    // 暂时返回默认权限，实际项目中应该从数据库查询
    // 例如：通过 role_permissions 关联表查询
    return ['user:read', 'user:write']
  }
}
