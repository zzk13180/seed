/**
 * JWT Payload 接口
 */
export interface JwtPayload {
  sub: number // 用户 ID
  username: string
  roles?: string[]
  permissions?: string[]
  jti?: string // JWT ID，用于黑名单
}

/**
 * 当前用户信息接口
 */
export interface CurrentUser {
  id: number
  username: string
  roles?: string[]
  permissions?: string[]
  jti?: string // JWT ID，用于登出验证
}
