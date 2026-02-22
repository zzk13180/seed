import { z } from 'zod'

/**
 * 环境变量校验 Schema
 *
 * 使用 Zod 在启动时验证所有必要的环境变量
 */
const envSchema = z.object({
  NODE_ENV: z.enum(['development', 'production', 'test']).default('development'),
  PORT: z.coerce.number().default(3003),

  // Neon PostgreSQL
  DATABASE_URL: z.string().url(),

  // Better Auth
  BETTER_AUTH_SECRET: z.string().min(1),
  BETTER_AUTH_URL: z.string().url().default('http://localhost:3003'),

  // Redis (optional)
  REDIS_URL: z.string().url().optional(),
})

export type Env = z.infer<typeof envSchema>

let _env: Env | undefined

/**
 * 获取已验证的环境变量（惰性初始化 + 缓存）
 */
export function env(): Env {
  if (!_env) {
    const result = envSchema.safeParse(process.env)
    if (!result.success) {
      console.error('❌ Invalid environment variables:')
      console.error(result.error.flatten().fieldErrors)
      process.exit(1)
    }
    _env = result.data
  }
  return _env
}

export const isDev = () => env().NODE_ENV === 'development'
export const isProd = () => env().NODE_ENV === 'production'
