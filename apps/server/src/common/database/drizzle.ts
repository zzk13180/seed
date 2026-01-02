import { drizzle, type NodePgDatabase } from 'drizzle-orm/node-postgres'
import { Pool } from 'pg'
import * as schema from './schema'

export type DrizzleDB = NodePgDatabase<typeof schema>

/**
 * 创建 Drizzle 数据库实例 (PostgreSQL)
 */
export async function createDrizzleConnection(config: {
  host: string
  port: number
  user: string
  password: string
  database: string
  logging?: boolean
}): Promise<{ db: DrizzleDB; pool: Pool }> {
  const pool = new Pool({
    host: config.host,
    port: config.port,
    user: config.user,
    password: config.password,
    database: config.database,
    max: 10,
    idleTimeoutMillis: 30_000,
    connectionTimeoutMillis: 2000,
  })

  const db = drizzle(pool, {
    schema,
    logger: config.logging,
  }) as DrizzleDB

  return { db, pool }
}
