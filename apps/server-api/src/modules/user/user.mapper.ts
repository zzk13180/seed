import { Injectable } from '@nestjs/common'
import { instanceToPlain, plainToInstance } from 'class-transformer'
import { User } from './entities/user.entity'
import { UserVo } from './vo/user.vo'
import { UserCreateDto } from './dto/user-create.dto'

/**
 * 用户对象转换器
 *
 * 负责 User Entity 与 UserVo、DTO 之间的转换
 * 确保敏感数据（如密码）不会泄露
 */
@Injectable()
export class UserMapper {
  /**
   * Entity 转 VO
   * 自动排除密码等敏感字段
   */
  toVO(user: User | null): UserVo | null {
    if (!user) {
      return null
    }

    const vo = plainToInstance(UserVo, user, {
      excludeExtraneousValues: false,
      enableImplicitConversion: true,
    })

    // 确保密码字段被移除（双重保障）
    delete (vo as any).password
    delete (vo as any).deleted

    return vo
  }

  /**
   * Entity 列表转 VO 列表
   */
  toVOList(users: User[]): UserVo[] {
    if (!users || users.length === 0) {
      return []
    }
    return users.map(user => this.toVO(user)!)
  }

  /**
   * CreateDTO 转 Entity（不包含 id）
   */
  toEntity(dto: UserCreateDto): Partial<User> {
    const entity = plainToInstance(User, dto, {
      excludeExtraneousValues: false,
    })

    // 不要在这里设置 id
    delete (entity as any).id

    return entity
  }

  /**
   * 更新 Entity（只更新非 undefined 字段，允许 null 用于清空字段）
   */
  updateEntity<T extends object>(entity: User, updateDto: T): User {
    const updateData = instanceToPlain(updateDto, {
      exposeUnsetFields: false,
    })

    // 只过滤 undefined 值，允许 null 用于清空字段（如头像、昵称等）
    Object.keys(updateData).forEach(key => {
      if (updateData[key] !== undefined) {
        ;(entity as any)[key] = updateData[key]
      }
    })

    return entity
  }

  /**
   * 构建登录返回的用户信息 VO
   * 明确排除所有敏感信息
   */
  toLoginUserVO(user: User): UserVo {
    return {
      id: user.id,
      username: user.username,
      nickname: user.nickname,
      email: user.email,
      phone: user.phone,
      avatar: user.avatar,
      status: user.status,
      createdAt: user.createdAt,
      updatedAt: user.updatedAt,
    } as UserVo
  }
}
