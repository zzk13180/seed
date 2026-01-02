import { Injectable, CanActivate, ExecutionContext } from '@nestjs/common'
import { Reflector } from '@nestjs/core'
import { PERMISSIONS_KEY } from '../decorators/permissions.decorator'
import { ForbiddenException } from '../exceptions/business.exception'

/**
 * 权限守卫
 */
@Injectable()
export class PermissionsGuard implements CanActivate {
  constructor(private readonly reflector: Reflector) {}

  canActivate(context: ExecutionContext): boolean {
    const requiredPermissions = this.reflector.getAllAndOverride<string[]>(PERMISSIONS_KEY, [
      context.getHandler(),
      context.getClass(),
    ])
    if (!requiredPermissions) {
      return true
    }
    const { user } = context.switchToHttp().getRequest()
    const hasPermission = requiredPermissions.some(permission =>
      user.permissions?.includes(permission),
    )

    if (!hasPermission) {
      throw ForbiddenException.insufficientPermissions()
    }

    return true
  }
}
