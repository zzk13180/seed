import type { IUserVo } from '../user/user'

/**
 * 登录参数 DTO
 */
export interface ILoginDto {
  /** 用户名 */
  username: string
  /** 密码 */
  password: string
}

/**
 * 登录响应 VO
 */
export interface ILoginVo {
  /** 访问令牌 */
  accessToken: string
  /** 刷新令牌 */
  refreshToken: string
  /** 用户信息 */
  user: IUserVo
  /** 过期时间（秒） */
  expiresIn: number
}

/**
 * 刷新令牌参数 DTO
 */
export interface IRefreshTokenDto {
  /** 刷新令牌 */
  refreshToken: string
}
