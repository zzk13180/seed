import { Injectable } from '@nestjs/common'
import { plainToInstance } from 'class-transformer'
import { UserVo } from './vo/user.vo'
import type { User } from '../../common/database/schema'

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
