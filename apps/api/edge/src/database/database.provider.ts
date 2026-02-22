import { createDatabase as _createDatabase, createDatabaseSingleton } from '@seed/db/provider'
import { env } from '../config/env.config'

export type { Database } from '@seed/db/provider'

/**
 * 数据库单例工厂（绑定当前运行时的环境变量获取方式）
 */
export const db = createDatabaseSingleton(() => env().DATABASE_URL)

/**
 * 创建新的数据库实例（用于特殊场景，如测试）
 */
export function createDatabase() {
  return _createDatabase(env().DATABASE_URL)
}
