import { pgTable, varchar, integer, timestamp, text, boolean } from 'drizzle-orm/pg-core'
import { sql } from 'drizzle-orm'

// ────────────────────────────────────────────────────
// Better Auth 核心表
// ────────────────────────────────────────────────────

/**
 * 用户表 — Better Auth 管理
 */
export const user = pgTable('user', {
  id: text('id').primaryKey(),
  name: text('name').notNull(),
  email: text('email').notNull().unique(),
  emailVerified: boolean('email_verified').notNull().default(false),
  image: text('image'),
  createdAt: timestamp('created_at', { mode: 'date' })
    .notNull()
    .default(sql`CURRENT_TIMESTAMP`),
  updatedAt: timestamp('updated_at', { mode: 'date' })
    .notNull()
    .default(sql`CURRENT_TIMESTAMP`),

  // 扩展字段
  phone: varchar('phone', { length: 20 }),
  role: text('role').notNull().default('user'),
  status: integer('status').notNull().default(1), // 0=disabled, 1=enabled
  deleted: integer('deleted').notNull().default(0),
})

/**
 * 会话表 — Better Auth 管理
 */
export const session = pgTable('session', {
  id: text('id').primaryKey(),
  expiresAt: timestamp('expires_at', { mode: 'date' }).notNull(),
  token: text('token').notNull().unique(),
  createdAt: timestamp('created_at', { mode: 'date' })
    .notNull()
    .default(sql`CURRENT_TIMESTAMP`),
  updatedAt: timestamp('updated_at', { mode: 'date' })
    .notNull()
    .default(sql`CURRENT_TIMESTAMP`),
  ipAddress: text('ip_address'),
  userAgent: text('user_agent'),
  userId: text('user_id')
    .notNull()
    .references(() => user.id),
})

/**
 * 账号表 — Better Auth 管理 (OAuth providers + credentials)
 */
export const account = pgTable('account', {
  id: text('id').primaryKey(),
  accountId: text('account_id').notNull(),
  providerId: text('provider_id').notNull(),
  userId: text('user_id')
    .notNull()
    .references(() => user.id),
  accessToken: text('access_token'),
  refreshToken: text('refresh_token'),
  idToken: text('id_token'),
  accessTokenExpiresAt: timestamp('access_token_expires_at', { mode: 'date' }),
  refreshTokenExpiresAt: timestamp('refresh_token_expires_at', { mode: 'date' }),
  scope: text('scope'),
  password: text('password'),
  createdAt: timestamp('created_at', { mode: 'date' })
    .notNull()
    .default(sql`CURRENT_TIMESTAMP`),
  updatedAt: timestamp('updated_at', { mode: 'date' })
    .notNull()
    .default(sql`CURRENT_TIMESTAMP`),
})

/**
 * 验证表 — Better Auth 管理 (email verification, password reset)
 */
export const verification = pgTable('verification', {
  id: text('id').primaryKey(),
  identifier: text('identifier').notNull(),
  value: text('value').notNull(),
  expiresAt: timestamp('expires_at', { mode: 'date' }).notNull(),
  createdAt: timestamp('created_at', { mode: 'date' }).default(sql`CURRENT_TIMESTAMP`),
  updatedAt: timestamp('updated_at', { mode: 'date' }).default(sql`CURRENT_TIMESTAMP`),
})

// ────────────────────────────────────────────────────
// 类型推断
// ────────────────────────────────────────────────────

export type User = typeof user.$inferSelect
export type NewUser = typeof user.$inferInsert
export type Session = typeof session.$inferSelect
export type Account = typeof account.$inferSelect
