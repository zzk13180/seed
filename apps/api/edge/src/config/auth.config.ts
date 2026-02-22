import { createAuth } from '@seed/kit/auth/server'
import * as schema from '@seed/db/schema'
import { createDatabase } from '../database/database.provider'
import { env } from './env.config'

/**
 * Better Auth 实例（消费 @seed/kit/auth/server 工厂）
 *
 * api/edge 使用 Neon HTTP 驱动，部署到 Cloudflare Workers
 */
export const auth = createAuth({
  db: createDatabase(),
  schema,
  secret: env().BETTER_AUTH_SECRET,
  baseURL: env().BETTER_AUTH_URL,
})

export type Auth = typeof auth
export type AuthSession = typeof auth.$Infer.Session
