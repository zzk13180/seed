import { Injectable, CanActivate, ExecutionContext } from '@nestjs/common'
import { Reflector } from '@nestjs/core'
import { ROLES_KEY } from '../decorators/roles.decorator'
import { ForbiddenException } from '../exceptions/business.exception'

/**
 * 角色守卫
 */
@Injectable()
export class RolesGuard implements CanActivate {
  constructor(private readonly reflector: Reflector) {}

  canActivate(context: ExecutionContext): boolean {
    const requiredRoles = this.reflector.getAllAndOverride<string[]>(ROLES_KEY, [
      context.getHandler(),
      context.getClass(),
    ])
    if (!requiredRoles) {
      return true
    }
    const { user } = context.switchToHttp().getRequest()
    const hasRole = requiredRoles.some(role => user.roles?.includes(role))

    if (!hasRole) {
      throw ForbiddenException.roleRequired(requiredRoles.join(' 或 '))
    }

    return true
  }
}
