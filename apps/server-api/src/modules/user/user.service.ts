import { Injectable } from '@nestjs/common'
import { InjectRepository } from '@nestjs/typeorm'
import { Repository, Brackets } from 'typeorm'
import * as bcrypt from 'bcrypt'
import { PageResultDto } from '../../common/dto/page-result.dto'
import { UserStatus } from '../../common/enums/user.enum'
import { UserException } from '../../common/exceptions/business.exception'
import { AuditSubscriber } from '../../common/subscribers/audit.subscriber'
import { escapeLikeString } from '../../common/utils/sql.utils'
import { User } from './entities/user.entity'
import { UserCreateDto } from './dto/user-create.dto'
import { UserUpdateDto } from './dto/user-update.dto'
import { UserQueryDto } from './dto/user-query.dto'
import { UserVo } from './vo/user.vo'
import { UserMapper } from './user.mapper'

/**
 * 用户服务
 */
@Injectable()
export class UserService {
  constructor(
    @InjectRepository(User)
    private readonly userRepository: Repository<User>,
    private readonly userMapper: UserMapper,
  ) {}

  /**
   * 分页查询用户
   */
  async page(query: UserQueryDto): Promise<PageResultDto<UserVo>> {
    const qb = this.userRepository
      .createQueryBuilder('user')
      .where('user.deleted = :deleted', { deleted: false })

    // 关键字搜索（用户名、昵称、邮箱 OR 条件）
    if (query.keyword) {
      const keyword = `%${escapeLikeString(query.keyword)}%`
      qb.andWhere(
        new Brackets(sub => {
          sub
            .where('user.username LIKE :keyword', { keyword })
            .orWhere('user.nickname LIKE :keyword', { keyword })
            .orWhere('user.email LIKE :keyword', { keyword })
        }),
      )
    }

    // 精确字段过滤（与关键字搜索独立，用于进一步筛选）
    if (query.username) {
      qb.andWhere('user.username LIKE :username', {
        username: `%${escapeLikeString(query.username)}%`,
      })
    }
    if (query.nickname) {
      qb.andWhere('user.nickname LIKE :nickname', {
        nickname: `%${escapeLikeString(query.nickname)}%`,
      })
    }
    if (query.email) {
      qb.andWhere('user.email LIKE :email', {
        email: `%${escapeLikeString(query.email)}%`,
      })
    }
    if (query.phone) {
      qb.andWhere('user.phone LIKE :phone', {
        phone: `%${escapeLikeString(query.phone)}%`,
      })
    }
    if (query.status !== undefined) {
      qb.andWhere('user.status = :status', { status: query.status })
    }

    // 排序和分页
    const orderClause = query.getUserOrderClause()
    for (const [field, direction] of Object.entries(orderClause)) {
      qb.addOrderBy(`user.${field}`, direction)
    }

    qb.skip(query.getSkip()).take(query.getTake())

    const [list, total] = await qb.getManyAndCount()

    const voList = this.userMapper.toVOList(list)
    return PageResultDto.create(voList, total, query.page, query.pageSize)
  }

  /**
   * 查询所有用户
   */
  async findAll(): Promise<UserVo[]> {
    const users = await this.userRepository.find({
      where: { deleted: false },
      order: { createdAt: 'DESC' },
    })
    return this.userMapper.toVOList(users)
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
    return this.userRepository.findOne({
      where: { username, deleted: false },
    })
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
      const existingEmail = await this.userRepository.findOne({
        where: { email: dto.email, deleted: false },
      })
      if (existingEmail) {
        throw UserException.emailExists(dto.email)
      }
    }

    // 密码加密
    const hashedPassword = await bcrypt.hash(dto.password, 10)

    const user = this.userRepository.create({
      ...dto,
      password: hashedPassword,
      status: UserStatus.ENABLED,
    })

    const savedUser = await this.userRepository.save(user)
    return this.userMapper.toVO(savedUser)!
  }

  /**
   * 更新用户
   */
  async update(id: number, dto: UserUpdateDto): Promise<UserVo> {
    const user = await this.findUserEntityById(id)

    // 检查邮箱是否已被其他用户使用
    if (dto.email && dto.email !== user.email) {
      const existingEmail = await this.userRepository.findOne({
        where: { email: dto.email, deleted: false },
      })
      if (existingEmail && existingEmail.id !== id) {
        throw UserException.emailExists(dto.email)
      }
    }

    this.userMapper.updateEntity(user, dto)
    const savedUser = await this.userRepository.save(user)
    return this.userMapper.toVO(savedUser)!
  }

  /**
   * 删除用户（软删除）
   */
  async delete(id: number): Promise<void> {
    const user = await this.findUserEntityById(id)
    user.deleted = true
    await this.userRepository.save(user)
  }

  /**
   * 批量删除用户
   */
  async deleteBatch(ids: number[]): Promise<void> {
    if (!ids || ids.length === 0) {
      return
    }
    // 使用 QueryBuilder 避免类型问题，并手动设置 updatedBy
    // 因为 TypeORM subscriber 的 beforeUpdate 不会在批量更新时触发
    const userId = AuditSubscriber.getCurrentUserId()
    await this.userRepository
      .createQueryBuilder()
      .update(User)
      .set({ deleted: true, updatedBy: userId ?? undefined })
      .whereInIds(ids)
      .execute()
  }

  /**
   * 更新用户状态
   */
  async updateStatus(id: number, status: UserStatus): Promise<void> {
    const user = await this.findUserEntityById(id)
    user.status = status
    await this.userRepository.save(user)
  }

  /**
   * 重置密码
   */
  async resetPassword(id: number, newPassword: string): Promise<void> {
    const user = await this.findUserEntityById(id)
    // 密码长度验证已在 ResetPasswordDto 中完成
    user.password = await bcrypt.hash(newPassword, 10)
    await this.userRepository.save(user)
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
    const user = await this.userRepository.findOne({
      where: { id, deleted: false },
    })
    if (!user) {
      throw UserException.notFound(id)
    }
    return user
  }
}
