import { Injectable, Inject } from '@nestjs/common'
import { eq, and, or, like, inArray, desc, asc, sql } from 'drizzle-orm'
import * as bcrypt from 'bcrypt'
import { PageResultDto } from '../../common/dto/page-result.dto'
import { UserStatus } from '../../common/enums/user.enum'
import { UserException } from '../../common/exceptions/business.exception'
import { AuditSubscriber } from '../../common/subscribers/audit.subscriber'
import { escapeLikeString } from '../../common/utils/sql.utils'
import { DRIZZLE_DB } from '../../common/database/drizzle.constants'
import { users, type User } from '../../common/database/schema'
import { UserCreateDto } from './dto/user-create.dto'
import { UserUpdateDto } from './dto/user-update.dto'
import { UserQueryDto } from './dto/user-query.dto'
import { UserVo } from './vo/user.vo'
import { UserMapper } from './user.mapper'
import type { DrizzleDB } from '../../common/database/drizzle'

/**
 * 用户服务
 */
@Injectable()
export class UserService {
  constructor(
    @Inject(DRIZZLE_DB)
    private readonly db: DrizzleDB,
    private readonly userMapper: UserMapper,
  ) {}

  /**
   * 分页查询用户
   */
  async page(query: UserQueryDto): Promise<PageResultDto<UserVo>> {
    const conditions = [eq(users.deleted, 0)]

    // 关键字搜索（用户名、昵称、邮箱 OR 条件）
    if (query.keyword) {
      const keyword = `%${escapeLikeString(query.keyword)}%`
      conditions.push(
        or(
          like(users.username, keyword),
          like(users.nickname, keyword),
          like(users.email, keyword),
        )!,
      )
    }

    // 精确字段过滤
    if (query.username) {
      conditions.push(like(users.username, `%${escapeLikeString(query.username)}%`))
    }
    if (query.nickname) {
      conditions.push(like(users.nickname, `%${escapeLikeString(query.nickname)}%`))
    }
    if (query.email) {
      conditions.push(like(users.email, `%${escapeLikeString(query.email)}%`))
    }
    if (query.phone) {
      conditions.push(like(users.phone, `%${escapeLikeString(query.phone)}%`))
    }
    if (query.status !== undefined) {
      conditions.push(eq(users.status, query.status))
    }

    // 排序
    const orderClause = query.getUserOrderClause()
    const orderByList: ReturnType<typeof asc>[] = []
    const columnMap: Record<
      string,
      typeof users.id | typeof users.createdAt | typeof users.updatedAt | typeof users.username
    > = {
      id: users.id,
      createdAt: users.createdAt,
      updatedAt: users.updatedAt,
      username: users.username,
    }
    for (const [field, direction] of Object.entries(orderClause)) {
      const column = columnMap[field]
      if (column) {
        orderByList.push(direction === 'ASC' ? asc(column) : desc(column))
      }
    }

    // 查询数据
    const list = await this.db
      .select()
      .from(users)
      .where(and(...conditions))
      .orderBy(...orderByList)
      .offset(query.getSkip())
      .limit(query.getTake())

    // 查询总数
    const countResult = await this.db
      .select({ count: sql<number>`count(*)` })
      .from(users)
      .where(and(...conditions))

    const total = Number(countResult[0]?.count ?? 0)

    const voList = this.userMapper.toVOList(list)
    return PageResultDto.create(voList, total, query.page, query.pageSize)
  }

  /**
   * 查询所有用户
   */
  async findAll(): Promise<UserVo[]> {
    const list = await this.db
      .select()
      .from(users)
      .where(eq(users.deleted, 0))
      .orderBy(desc(users.createdAt))

    return this.userMapper.toVOList(list)
  }

  /**
   * 根据 ID 查询用户
   */
  async findById(id: number): Promise<UserVo> {
    const user = await this.findUserEntityById(id)
    return this.userMapper.toVO(user)!
  }

  /**
   * 根据用户名查询用户
   */
  async findByUsername(username: string): Promise<User | null> {
    const result = await this.db
      .select()
      .from(users)
      .where(and(eq(users.username, username), eq(users.deleted, 0)))
      .limit(1)

    return result[0] ?? null
  }

  /**
   * 创建用户
   */
  async create(dto: UserCreateDto): Promise<UserVo> {
    // 检查用户名是否已存在
    const existingUser = await this.findByUsername(dto.username)
    if (existingUser) {
      throw UserException.usernameExists(dto.username)
    }

    // 检查邮箱是否已存在
    if (dto.email) {
      const existingEmail = await this.db
        .select()
        .from(users)
        .where(and(eq(users.email, dto.email), eq(users.deleted, 0)))
        .limit(1)

      if (existingEmail[0]) {
        throw UserException.emailExists(dto.email)
      }
    }

    // 密码加密
    const hashedPassword = await bcrypt.hash(dto.password, 10)

    // 获取当前用户 ID 用于审计
    const currentUserId = AuditSubscriber.getCurrentUserId() ?? undefined

    const result = await this.db.insert(users).values({
      username: dto.username,
      password: hashedPassword,
      nickname: dto.nickname,
      email: dto.email,
      phone: dto.phone,
      avatar: dto.avatar,
      status: UserStatus.ENABLED,
      createdBy: currentUserId,
      updatedBy: currentUserId,
    })

    const insertId = Number(result[0].insertId)
    const savedUser = await this.findUserEntityById(insertId)
    return this.userMapper.toVO(savedUser)!
  }

  /**
   * 更新用户
   */
  async update(id: number, dto: UserUpdateDto): Promise<UserVo> {
    const user = await this.findUserEntityById(id)

    // 检查邮箱是否已被其他用户使用
    if (dto.email && dto.email !== user.email) {
      const existingEmail = await this.db
        .select()
        .from(users)
        .where(and(eq(users.email, dto.email), eq(users.deleted, 0)))
        .limit(1)

      if (existingEmail[0] && existingEmail[0].id !== id) {
        throw UserException.emailExists(dto.email)
      }
    }

    // 获取当前用户 ID 用于审计
    const currentUserId = AuditSubscriber.getCurrentUserId() ?? undefined

    // 构建更新对象
    const updateData: Partial<User> = {
      updatedBy: currentUserId,
    }

    if (dto.nickname !== undefined) updateData.nickname = dto.nickname
    if (dto.email !== undefined) updateData.email = dto.email
    if (dto.phone !== undefined) updateData.phone = dto.phone
    if (dto.avatar !== undefined) updateData.avatar = dto.avatar

    await this.db.update(users).set(updateData).where(eq(users.id, id))

    const savedUser = await this.findUserEntityById(id)
    return this.userMapper.toVO(savedUser)!
  }

  /**
   * 删除用户（软删除）
   */
  async delete(id: number): Promise<void> {
    await this.findUserEntityById(id) // 确保用户存在

    const currentUserId = AuditSubscriber.getCurrentUserId() ?? undefined
    await this.db
      .update(users)
      .set({ deleted: 1, updatedBy: currentUserId })
      .where(eq(users.id, id))
  }

  /**
   * 批量删除用户
   */
  async deleteBatch(ids: number[]): Promise<void> {
    if (!ids || ids.length === 0) {
      return
    }

    const currentUserId = AuditSubscriber.getCurrentUserId() ?? undefined
    await this.db
      .update(users)
      .set({ deleted: 1, updatedBy: currentUserId })
      .where(inArray(users.id, ids))
  }

  /**
   * 更新用户状态
   */
  async updateStatus(id: number, status: UserStatus): Promise<void> {
    await this.findUserEntityById(id) // 确保用户存在

    const currentUserId = AuditSubscriber.getCurrentUserId() ?? undefined
    await this.db.update(users).set({ status, updatedBy: currentUserId }).where(eq(users.id, id))
  }

  /**
   * 重置密码
   */
  async resetPassword(id: number, newPassword: string): Promise<void> {
    await this.findUserEntityById(id) // 确保用户存在

    const hashedPassword = await bcrypt.hash(newPassword, 10)
    const currentUserId = AuditSubscriber.getCurrentUserId() ?? undefined

    await this.db
      .update(users)
      .set({ password: hashedPassword, updatedBy: currentUserId })
      .where(eq(users.id, id))
  }

  /**
   * 验证密码
   */
  async validatePassword(user: User, password: string): Promise<boolean> {
    return bcrypt.compare(password, user.password)
  }

  /**
   * 根据 ID 查询用户实体（用于内部服务调用，如 AuthService）
   * 返回完整的 User Entity，包含密码等敏感信息
   */
  async findEntityById(id: number): Promise<User> {
    return this.findUserEntityById(id)
  }

  /**
   * 内部方法：根据 ID 查询用户实体
   */
  private async findUserEntityById(id: number): Promise<User> {
    const result = await this.db
      .select()
      .from(users)
      .where(and(eq(users.id, id), eq(users.deleted, 0)))
      .limit(1)

    const user = result[0]
    if (!user) {
      throw UserException.notFound(id)
    }
    return user
  }
}
