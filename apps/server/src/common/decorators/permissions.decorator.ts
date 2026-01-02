import { SetMetadata } from '@nestjs/common'

export const PERMISSIONS_KEY = 'permissions'
/**
 * 权限装饰器
 */
export const Permissions = (...permissions: string[]) => SetMetadata(PERMISSIONS_KEY, permissions)
