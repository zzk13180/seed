import { z } from 'zod'

// ============================================================
// 登录 Schema
// ============================================================

/**
 * 登录参数 Schema
 */
export const loginSchema = z.object({
  email: z.string().email(),
  password: z.string().min(6),
})

/**
 * 注册参数 Schema
 */
export const signUpSchema = z.object({
  name: z.string().min(1).max(50),
  email: z.string().email(),
  password: z.string().min(6).max(64),
})

// ────────────────────────────────────────────────────
// TypeScript 类型推导
// ────────────────────────────────────────────────────

export type LoginDto = z.infer<typeof loginSchema>
export type SignUpDto = z.infer<typeof signUpSchema>

/**
 * 刷新令牌参数 DTO
 */
export interface RefreshTokenDto {
  refreshToken: string
}
