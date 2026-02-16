import { createPageResult } from '@seed/contracts/schemas/common'
import { NotFoundError, ConflictError } from '@seed/contracts/errors'
import { UserRepository } from './user.repository'
import { toUserVO, type UserVO } from './user.vo'
import type { PageResult } from '@seed/contracts/schemas/common'
import type { AnyDatabase } from '@seed/db'
import type { UserQuery, UserUpdateDto } from '@seed/contracts/schemas/user'

// Bun global type (available when running on Bun runtime)
declare const Bun: { password: { hash: (pw: string, opts?: any) => Promise<string> } }

/** 密码哈希函数签名（跨运行时兼容） */
export type HashPasswordFn = (password: string) => Promise<string>

/**
 * 用户服务 — 封装业务逻辑
 *
 * 职责边界：
 * - Controller: 路由定义、请求验证、响应构建
 * - Service:    业务逻辑、异常抛出、VO 转换
 * - Repository: 数据访问、ORM 操作
 */
export class UserService {
  private readonly userRepository: UserRepository
  private readonly hashPassword: HashPasswordFn

  constructor(db: AnyDatabase, hashPassword?: HashPasswordFn) {
    this.hashPassword =
      hashPassword ??
      (async pw => {
        // 默认使用 Bun.password.hash（仅 Bun 运行时可用）
        return Bun.password.hash(pw, { algorithm: 'bcrypt', cost: 10 })
      })
    this.userRepository = new UserRepository(db)
  }

  /**
   * 分页查询用户
   */
  async findPage(query: UserQuery): Promise<PageResult<UserVO>> {
    const { list, total } = await this.userRepository.findPage(query)
    return createPageResult(
      list.map(item => toUserVO(item)),
      total,
      query.page,
      query.pageSize,
    )
  }

  /**
   * 查询所有用户
   */
  async findAll(): Promise<UserVO[]> {
    const list = await this.userRepository.findAll()
    return list.map(item => toUserVO(item))
  }

  /**
   * 根据 ID 查询用户
   */
  async findById(id: string): Promise<UserVO> {
    const entity = await this.userRepository.findById(id)
    if (!entity) {
      throw new NotFoundError('User', id)
    }
    return toUserVO(entity)
  }

  /**
   * 更新用户
   */
  async update(id: string, dto: UserUpdateDto): Promise<UserVO> {
    const existing = await this.userRepository.findById(id)
    if (!existing) {
      throw new NotFoundError('User', id)
    }

    // 邮箱唯一性检查
    if (dto.email && dto.email !== existing.email) {
      const conflict = await this.userRepository.findByEmail(dto.email, id)
      if (conflict) {
        throw ConflictError.emailExists(dto.email)
      }
    }

    // 构建更新数据
    const updateData: Record<string, unknown> = {}
    if (dto.nickname !== undefined) updateData.name = dto.nickname
    if (dto.email !== undefined) updateData.email = dto.email
    if (dto.phone !== undefined) updateData.phone = dto.phone
    if (dto.avatar !== undefined) updateData.image = dto.avatar

    const updated = await this.userRepository.update(id, updateData)
    return toUserVO(updated)
  }

  /**
   * 删除用户（软删除）
   */
  async delete(id: string): Promise<void> {
    const existing = await this.userRepository.findById(id)
    if (!existing) {
      throw new NotFoundError('User', id)
    }
    await this.userRepository.softDelete(id)
  }

  /**
   * 批量删除用户
   */
  async batchDelete(ids: string[]): Promise<void> {
    await this.userRepository.batchSoftDelete(ids)
  }

  /**
   * 更新用户状态
   */
  async updateStatus(id: string, status: number): Promise<void> {
    const existing = await this.userRepository.findById(id)
    if (!existing) {
      throw new NotFoundError('User', id)
    }
    await this.userRepository.updateStatus(id, status)
  }

  /**
   * 重置用户密码
   */
  async resetPassword(id: string, password: string): Promise<void> {
    const existing = await this.userRepository.findById(id)
    if (!existing) {
      throw new NotFoundError('User', id)
    }

    const hashedPassword = await this.hashPassword(password)
    await this.userRepository.resetPassword(id, hashedPassword)
  }
}
