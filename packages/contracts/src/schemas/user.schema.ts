import { z } from 'zod'
import { UserStatus } from '../enums/user.enum'
import { pageRequestSchema } from './common.schema'

/**
 * 用户模块 Zod Schemas — 单一真实来源 (SSOT)
 *
 * 前端和后端共同消费此文件。
 * 前端用于表单验证 + 类型推导，后端用于请求校验。
 */

// ────────────────────────────────────────────────────
// VO（View Object）
// ────────────────────────────────────────────────────

export const userVoSchema = z.object({
  id: z.string(),
  name: z.string(),
  email: z.string().email().nullable(),
  emailVerified: z.boolean(),
  image: z.string().nullable(),
  phone: z.string().nullable(),
  role: z.string(),
  status: z.number().int(),
  createdAt: z.string(),
  updatedAt: z.string(),
})

export type UserVO = z.infer<typeof userVoSchema>

// ────────────────────────────────────────────────────
// 查询
// ────────────────────────────────────────────────────

export const userQuerySchema = pageRequestSchema.extend({
  keyword: z.string().optional(),
  username: z.string().optional(),
  nickname: z.string().optional(),
  email: z.string().optional(),
  phone: z.string().optional(),
  status: z.coerce.number().int().min(0).max(1).optional(),
})

export type UserQuery = z.infer<typeof userQuerySchema>

// ────────────────────────────────────────────────────
// 创建
// ────────────────────────────────────────────────────

export const userCreateSchema = z.object({
  username: z
    .string()
    .min(3)
    .max(50)
    .regex(/^\w+$/, 'Username can only contain letters, numbers and underscores'),
  password: z.string().min(6).max(50),
  nickname: z.string().max(50).optional(),
  email: z.string().email().optional(),
  phone: z
    .string()
    .regex(/^1[3-9]\d{9}$/, 'Invalid phone number')
    .optional(),
  avatar: z.string().url().optional(),
})

export type UserCreateDto = z.infer<typeof userCreateSchema>

// ────────────────────────────────────────────────────
// 更新
// ────────────────────────────────────────────────────

export const userUpdateSchema = z.object({
  nickname: z.string().max(50).optional(),
  email: z.string().email().optional(),
  phone: z
    .string()
    .regex(/^1[3-9]\d{9}$/, 'Invalid phone number')
    .optional(),
  avatar: z.string().url().optional(),
})

export type UserUpdateDto = z.infer<typeof userUpdateSchema>

// ────────────────────────────────────────────────────
// 状态更新 / 密码重置 / 批量删除
// ────────────────────────────────────────────────────

export const updateStatusSchema = z.object({
  status: z.nativeEnum(UserStatus),
})

export type UpdateStatusDto = z.infer<typeof updateStatusSchema>

export const resetPasswordSchema = z.object({
  password: z.string().min(6).max(32),
})

export type ResetPasswordDto = z.infer<typeof resetPasswordSchema>

export const batchDeleteUserSchema = z.object({
  ids: z.array(z.string()).min(1).max(100),
})

// ────────────────────────────────────────────────────
// 路径参数
// ────────────────────────────────────────────────────

export const idParamSchema = z.object({
  id: z.string(),
})
