import { Injectable, ExecutionContext, Inject } from '@nestjs/common'
import { Reflector } from '@nestjs/core'
import { AuthGuard } from '@nestjs/passport'
import { CACHE_MANAGER } from '@nestjs/cache-manager'
import { IS_PUBLIC_KEY } from '../decorators/public.decorator'
import { UnauthorizedException } from '../exceptions/business.exception'
import type { Cache } from 'cache-manager'

/**
 * JWT 认证守卫
 */
@Injectable()
export class JwtAuthGuard extends AuthGuard('jwt') {
  constructor(
    private readonly reflector: Reflector,
    @Inject(CACHE_MANAGER) private readonly cacheManager: Cache,
  ) {
    super()
  }

  async canActivate(context: ExecutionContext): Promise<boolean> {
    // 检查是否为公开接口
    const isPublic = this.reflector.getAllAndOverride<boolean>(IS_PUBLIC_KEY, [
      context.getHandler(),
      context.getClass(),
    ])
    if (isPublic) {
      return true
    }

    // 调用父类的 canActivate 进行 JWT 验证
    const result = await super.canActivate(context)
    if (!result) {
      return false
    }

    // 检查 token 是否在黑名单中
    const request = context.switchToHttp().getRequest()
    const user = request.user

    if (user?.jti) {
      const isBlacklisted = await this.cacheManager.get(`blacklist:token:${user.jti}`)
      if (isBlacklisted) {
        throw UnauthorizedException.tokenInvalid()
      }
    }

    return true
  }
}
