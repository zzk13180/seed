import { z } from 'zod'

// ============================================================
// 用户状态
// ============================================================

export const UserStatusValues = [0, 1] as const

export enum UserStatus {
  DISABLED = 0,
  ENABLED = 1,
}

// ============================================================
// 用户权限
// ============================================================

export const permissionValues = ['user:read', 'user:write', 'user:delete'] as const
export const permissionSchema = z.enum(permissionValues)
export type PermissionType = z.infer<typeof permissionSchema>

export enum Permission {
  USER_READ = 'user:read',
  USER_WRITE = 'user:write',
  USER_DELETE = 'user:delete',
}

// ============================================================
// 用户角色
// ============================================================

export const roleValues = ['admin', 'user'] as const
export const roleSchema = z.enum(roleValues)
export type RoleType = z.infer<typeof roleSchema>

export enum Role {
  ADMIN = 'admin',
  USER = 'user',
}
