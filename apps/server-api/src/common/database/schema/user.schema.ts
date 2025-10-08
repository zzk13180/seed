import { pgTable, bigserial, varchar, integer, timestamp, index } from 'drizzle-orm/pg-core'
import { sql } from 'drizzle-orm'
import { UserStatus } from '../../enums/user.enum'

/**
 * 用户表 Schema
 */
export const users = pgTable(
  't_user',
  {
    id: bigserial('id', { mode: 'number' }).primaryKey(),

    username: varchar('username', { length: 50 }).notNull().unique(),
    password: varchar('password', { length: 255 }).notNull(),
    nickname: varchar('nickname', { length: 50 }),
    email: varchar('email', { length: 100 }).unique(),
    phone: varchar('phone', { length: 20 }),
    avatar: varchar('avatar', { length: 255 }),
    status: integer('status').notNull().default(UserStatus.ENABLED),

    // 审计字段
    createdAt: timestamp('created_at', { mode: 'date' })
      .notNull()
      .default(sql`CURRENT_TIMESTAMP`),
    createdBy: bigserial('created_by', { mode: 'number' }),
    updatedAt: timestamp('updated_at', { mode: 'date' })
      .notNull()
      .default(sql`CURRENT_TIMESTAMP`)
      .$onUpdate(() => new Date()),
    updatedBy: bigserial('updated_by', { mode: 'number' }),
    deleted: integer('deleted').notNull().default(0),
  },
  table => [
    index('idx_username').on(table.username),
    index('idx_email').on(table.email),
    index('idx_phone').on(table.phone),
    index('idx_status').on(table.status),
  ],
)

/**
 * 用户表类型推断
 */
export type User = typeof users.$inferSelect
export type NewUser = typeof users.$inferInsert
