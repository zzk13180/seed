import { ApiProperty, ApiPropertyOptional } from '@nestjs/swagger'
import { UserStatus } from '../../../common/enums/user.enum'
import type { IUserVo, ISODateString } from '@seed/api-types'

/**
 * 用户响应 VO
 *
 * 实现 @seed/api-types 中的 IUserVo 接口
 * 注意：不应包含敏感字段（如 password），安全过滤在 UserMapper 中完成
 */
export class UserVo implements IUserVo {
  @ApiProperty({ description: '用户ID' })
  id!: number

  @ApiProperty({ description: '用户名' })
  username!: string

  @ApiPropertyOptional({ description: '昵称' })
  nickname!: string | null

  @ApiPropertyOptional({ description: '邮箱' })
  email!: string | null

  @ApiPropertyOptional({ description: '手机号' })
  phone!: string | null

  @ApiPropertyOptional({ description: '头像URL' })
  avatar!: string | null

  @ApiProperty({ description: '状态', enum: UserStatus })
  status!: UserStatus

  @ApiProperty({ description: '创建时间' })
  createdAt!: ISODateString

  @ApiProperty({ description: '更新时间' })
  updatedAt!: ISODateString
}
