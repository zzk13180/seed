// Schema
export { user, session, account, verification } from './schema/index'
export type { User, NewUser, Session, Account } from './schema/index'

// Provider
export { createDatabase, createDatabaseSingleton } from './provider'
export type { Database } from './provider'

// Common database type (works with any PG driver: Neon HTTP, node-postgres, etc.)
export type { AnyDatabase } from './types'
