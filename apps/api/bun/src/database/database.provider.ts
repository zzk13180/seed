import { drizzle, type NodePgDatabase } from 'drizzle-orm/node-postgres'
import { Pool } from 'pg'
import * as schema from '@seed/db/schema'
import { env } from '../config/env.config'

export type Database = NodePgDatabase<typeof schema>

let _pool: Pool | undefined
let _db: Database | undefined

/**
 * Get or create a singleton pg Pool + Drizzle instance (TCP connection).
 * Unlike api/edge which uses Neon HTTP driver, api/bun uses TCP for
 * long-running connections, WebSocket sessions, and Cron jobs.
 */
export function createDatabase(): Database {
  if (!_db) {
    const e = env()
    _pool = new Pool({
      host: e.DB_HOST,
      port: e.DB_PORT,
      user: e.DB_USERNAME,
      password: e.DB_PASSWORD,
      database: e.DB_DATABASE,
      max: 10,
      idleTimeoutMillis: 30_000,
      connectionTimeoutMillis: 2000,
    })
    _db = drizzle(_pool, { schema, logger: e.DB_LOGGING }) as Database
  }
  return _db
}

export function getPool(): Pool | undefined {
  return _pool
}

export const db = createDatabase
