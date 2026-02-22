import type { PgDatabase } from 'drizzle-orm/pg-core'
import type * as schema from './schema/index'

/**
 * 通用数据库类型 — 兼容所有 PostgreSQL 驱动
 *
 * 两个后端 app 使用不同的 Drizzle 驱动:
 * - apps/api/edge/  → Neon HTTP (NeonHttpDatabase, Serverless 适配)
 * - apps/api/bun/ → node-postgres TCP (NodePgDatabase, 长连接)
 *
 * 两者都继承自 PgDatabase，因此共享模块使用此类型即可同时兼容。
 *
 * @example
 * ```typescript
 * import type { AnyDatabase } from '@seed/db'
 *
 * export function createUserRoutes(db: AnyDatabase) {
 *   // db.select().from(user).where(...)  — 正常使用所有 Drizzle 查询
 * }
 * ```
 */

export type AnyDatabase = PgDatabase<any, typeof schema>
