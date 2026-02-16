import { neon } from '@neondatabase/serverless'
import { drizzle, type NeonHttpDatabase } from 'drizzle-orm/neon-http'
import * as schema from './schema/index'

export type Database = NeonHttpDatabase<typeof schema>

/**
 * 创建 Drizzle 数据库实例（Neon HTTP 驱动，适配 Serverless）
 *
 * @param databaseUrl - Neon PostgreSQL 连接字符串
 */
export function createDatabase(databaseUrl: string): Database {
  const sql = neon(databaseUrl)
  return drizzle(sql, { schema })
}

/**
 * 创建单例数据库实例工厂
 *
 * 适用于 Serverless 环境，避免冷启动重复创建连接。
 * 调用者传入延迟获取的 databaseUrl（从环境变量读取）。
 */
export function createDatabaseSingleton(getDatabaseUrl: () => string) {
  let _db: Database | undefined

  return function db(): Database {
    if (!_db) {
      _db = createDatabase(getDatabaseUrl())
    }
    return _db
  }
}
