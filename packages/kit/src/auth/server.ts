/**
 * Better Auth 服务端配置工厂
 *
 * 用法：
 *   import { createAuth } from '@seed/kit/auth/server'
 *   const auth = createAuth({ db, secret, baseURL })
 *
 * 此文件提供配置工厂，不直接创建 auth 实例——
 * 因为不同运行时（Workers / Bun / Node）的 DB 驱动和环境变量获取方式不同。
 */
import { betterAuth } from 'better-auth'
import { drizzleAdapter } from 'better-auth/adapters/drizzle'

export interface AuthServerOptions {
  /** Drizzle ORM 实例 */
  db: Parameters<typeof drizzleAdapter>[0]
  /** Drizzle schema（user, session, account, verification 表） */
  schema: Record<string, unknown>
  /** 签名密钥 */
  secret: string
  /** API 基础 URL（如 http://localhost:3003） */
  baseURL: string
  /** 允许的前端来源 */
  trustedOrigins?: string[]
}

/**
 * 创建 Better Auth 服务端实例
 */
export function createAuth(options: AuthServerOptions) {
  return betterAuth({
    database: drizzleAdapter(options.db, {
      provider: 'pg',
      schema: options.schema,
    }),
    secret: options.secret,
    baseURL: options.baseURL,
    basePath: '/api/auth',
    emailAndPassword: {
      enabled: true,
      minPasswordLength: 6,
      maxPasswordLength: 64,
    },
    session: {
      expiresIn: 60 * 60 * 24 * 7, // 7 天
      updateAge: 60 * 60 * 24, // 24 小时续期
      cookieCache: {
        enabled: true,
        maxAge: 5 * 60, // 5 分钟 cookie 缓存
      },
    },
    user: {
      additionalFields: {
        phone: { type: 'string', required: false },
        role: { type: 'string', required: false, defaultValue: 'user', input: false },
        status: { type: 'number', required: false, defaultValue: 1, input: false },
        deleted: { type: 'number', required: false, defaultValue: 0, input: false },
      },
    },
    trustedOrigins: options.trustedOrigins ?? [
      'http://localhost:5173', // Admin dev
      'http://localhost:5174', // Mobile dev
      'tauri://localhost', // Desktop
    ],
    advanced: {
      database: {
        generateId: () => crypto.randomUUID(),
      },
    },
  })
}

export type Auth = ReturnType<typeof createAuth>
