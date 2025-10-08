import { createParamDecorator } from '@nestjs/common'
import type { ExecutionContext } from '@nestjs/common'
import type { CurrentUser } from '../interfaces/auth.interface'

/**
 * 获取当前用户信息装饰器
 */
export const GetUser = createParamDecorator(
  (data: keyof CurrentUser | undefined, ctx: ExecutionContext): unknown => {
    const request = ctx.switchToHttp().getRequest<{ user?: CurrentUser }>()
    const user = request.user

    if (data) {
      return user?.[data]
    }
    return user
  },
)
