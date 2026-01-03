import type { ISODateString, IPageRequest } from '../common/response'
import type { UserStatus } from '../enums/user.enum'

/**
 * 用户信息 VO（View Object）
 * 用于 API 响应中的用户数据
 */
export interface IUserVo {
  /** 用户ID */
  id: number
  /** 用户名 */
  username: string
  /** 昵称 */
  nickname: string | null
  /** 邮箱 */
  email: string | null
  /** 手机号 */
  phone: string | null
  /** 头像URL */
  avatar: string | null
  /** 状态 */
  status: UserStatus
  /** 创建时间 */
  createdAt: ISODateString
  /** 更新时间 */
  updatedAt: ISODateString
}

/**
 * 用户创建参数 DTO
 */
export interface IUserCreateDto {
  /** 用户名 */
  username: string
  /** 密码 */
  password: string
  /** 昵称 */
  nickname?: string
  /** 邮箱 */
  email?: string
  /** 手机号 */
  phone?: string
  /** 头像URL */
  avatar?: string
}

/**
 * 用户更新参数 DTO
 */
export interface IUserUpdateDto {
  /** 昵称 */
  nickname?: string
  /** 邮箱 */
  email?: string
  /** 手机号 */
  phone?: string
  /** 头像URL */
  avatar?: string
}

/**
 * 用户查询参数 DTO
 */
export interface IUserQueryDto extends IPageRequest {
  /** 用户名（模糊匹配） */
  username?: string
  /** 昵称（模糊匹配） */
  nickname?: string
  /** 邮箱（模糊匹配） */
  email?: string
  /** 手机号（模糊匹配） */
  phone?: string
  /** 状态 */
  status?: UserStatus
}

/**
 * 更新用户状态参数 DTO
 */
export interface IUpdateStatusDto {
  /** 用户状态 */
  status: UserStatus
}

/**
 * 重置密码参数 DTO
 */
export interface IResetPasswordDto {
  /** 新密码 */
  password: string
}
