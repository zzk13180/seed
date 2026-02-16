import { eq, and, or, like, inArray, desc, asc, sql } from 'drizzle-orm'
import { user, account } from '@seed/db/schema'
import { escapeLikeString } from '@seed/kit/utils'
import type { AnyDatabase } from '@seed/db'
import type { User } from '@seed/db/schema'
import type { UserQuery } from '@seed/contracts/schemas/user'

/** 可排序字段白名单（防 SQL 注入） */
const SORTABLE_FIELDS = [
  'id',
  'username',
  'nickname',
  'email',
  'status',
  'createdAt',
  'updatedAt',
] as const

const COLUMN_MAP: Record<string, any> = {
  id: user.id,
  username: user.name,
  nickname: user.name,
  email: user.email,
  status: user.status,
  createdAt: user.createdAt,
  updatedAt: user.updatedAt,
}

/**
 * 用户仓储 — 封装所有 Drizzle ORM 数据访问操作
 *
 * 通过构造函数注入 Database 实例，兼容 Neon HTTP / node-postgres 驱动
 */
export class UserRepository {
  constructor(private readonly db: AnyDatabase) {}

  /**
   * 分页查询（带条件过滤 + 排序）
   */
  async findPage(query: UserQuery): Promise<{ list: User[]; total: number }> {
    const conditions = this.buildConditions(query)
    const { orderFn, orderColumn } = this.buildOrder(query)
    const offset = (query.page - 1) * query.pageSize

    const [list, countResult] = await Promise.all([
      this.db
        .select()
        .from(user)
        .where(and(...conditions))
        .orderBy(orderFn(orderColumn))
        .offset(offset)
        .limit(query.pageSize),
      this.db
        .select({ count: sql<number>`count(*)` })
        .from(user)
        .where(and(...conditions)),
    ])

    return {
      list,
      total: Number(countResult[0]?.count ?? 0),
    }
  }

  /**
   * 查询所有未删除用户
   */
  async findAll(): Promise<User[]> {
    return this.db.select().from(user).where(eq(user.deleted, 0)).orderBy(desc(user.createdAt))
  }

  /**
   * 根据 ID 查询单个用户
   */
  async findById(id: string): Promise<User | undefined> {
    const result = await this.db
      .select()
      .from(user)
      .where(and(eq(user.id, id), eq(user.deleted, 0)))
      .limit(1)
    return result[0]
  }

  /**
   * 根据邮箱查询用户（排除指定 ID）
   */
  async findByEmail(email: string, excludeId?: string): Promise<User | undefined> {
    const conditions = [eq(user.email, email), eq(user.deleted, 0)]
    const result = await this.db
      .select()
      .from(user)
      .where(and(...conditions))
      .limit(1)

    if (result[0] && excludeId && result[0].id === excludeId) {
      return undefined
    }
    return result[0]
  }

  /**
   * 更新用户字段
   */
  async update(id: string, data: Record<string, unknown>): Promise<User> {
    await this.db
      .update(user)
      .set({ ...data, updatedAt: new Date() })
      .where(eq(user.id, id))

    const updated = await this.db.select().from(user).where(eq(user.id, id)).limit(1)
    return updated[0]!
  }

  /**
   * 软删除
   */
  async softDelete(id: string): Promise<void> {
    await this.db.update(user).set({ deleted: 1, updatedAt: new Date() }).where(eq(user.id, id))
  }

  /**
   * 批量软删除
   */
  async batchSoftDelete(ids: string[]): Promise<void> {
    await this.db
      .update(user)
      .set({ deleted: 1, updatedAt: new Date() })
      .where(inArray(user.id, ids))
  }

  /**
   * 更新用户状态
   */
  async updateStatus(id: string, status: number): Promise<void> {
    await this.db.update(user).set({ status, updatedAt: new Date() }).where(eq(user.id, id))
  }

  /**
   * 重置密码（操作 account 表）
   */
  async resetPassword(userId: string, hashedPassword: string): Promise<void> {
    await this.db
      .update(account)
      .set({ password: hashedPassword, updatedAt: new Date() })
      .where(and(eq(account.userId, userId), eq(account.providerId, 'credential')))
  }

  private buildConditions(query: UserQuery) {
    const conditions = [eq(user.deleted, 0)]

    if (query.keyword) {
      const kw = `%${escapeLikeString(query.keyword)}%`
      conditions.push(or(like(user.name, kw), like(user.email, kw))!)
    }
    if (query.username) {
      conditions.push(like(user.name, `%${escapeLikeString(query.username)}%`))
    }
    if (query.nickname) {
      conditions.push(like(user.name, `%${escapeLikeString(query.nickname)}%`))
    }
    if (query.email) {
      conditions.push(like(user.email, `%${escapeLikeString(query.email)}%`))
    }
    if (query.phone) {
      conditions.push(like(user.phone, `%${escapeLikeString(query.phone)}%`))
    }
    if (query.status !== undefined) {
      conditions.push(eq(user.status, query.status))
    }

    return conditions
  }

  private buildOrder(query: UserQuery) {
    const orderField =
      query.orderBy && SORTABLE_FIELDS.includes(query.orderBy as (typeof SORTABLE_FIELDS)[number])
        ? query.orderBy
        : 'createdAt'
    const orderColumn = COLUMN_MAP[orderField] ?? user.createdAt
    const orderFn = query.orderDirection === 'ASC' ? asc : desc
    return { orderFn, orderColumn }
  }
}
