/**
 * 用户状态枚举
 */
export enum UserStatus {
  DISABLED = 0,
  ENABLED = 1,
}

/**
 * 用户权限
 */
export enum Permission {
  USER_READ = 'user:read',
  USER_WRITE = 'user:write',
  USER_DELETE = 'user:delete',
}

/**
 * 用户角色
 */
export enum Role {
  ADMIN = 'ADMIN',
  USER = 'USER',
}
