import { ApiPropertyOptional } from '@nestjs/swagger'
import { IsOptional, IsString, IsEnum } from 'class-validator'
import { PageRequestDto } from '../../../common/dto/page-request.dto'
import { UserStatus } from '../../../common/enums/user.enum'
import type { IUserQueryDto } from '@seed/api-types'

/**
 * 用户可排序的字段
 */
export const USER_SORTABLE_FIELDS = [
  'id',
  'username',
  'nickname',
  'email',
  'status',
  'createdAt',
  'updatedAt',
]

/**
 * 用户查询 DTO
 *
 * 实现 @seed/api-types 中的 IUserQueryDto 接口
 */
export class UserQueryDto extends PageRequestDto implements IUserQueryDto {
  @ApiPropertyOptional({ description: '用户名（模糊匹配）' })
  @IsOptional()
  @IsString()
  username?: string

  @ApiPropertyOptional({ description: '昵称（模糊匹配）' })
  @IsOptional()
  @IsString()
  nickname?: string

  @ApiPropertyOptional({ description: '邮箱（模糊匹配）' })
  @IsOptional()
  @IsString()
  email?: string

  @ApiPropertyOptional({ description: '手机号（模糊匹配）' })
  @IsOptional()
  @IsString()
  phone?: string

  @ApiPropertyOptional({ description: '状态', enum: UserStatus })
  @IsOptional()
  @IsEnum(UserStatus)
  status?: UserStatus

  /**
   * 获取用户排序配置
   */
  getUserOrderClause(): Record<string, 'ASC' | 'DESC'> {
    return this.getOrderClause(USER_SORTABLE_FIELDS, 'createdAt')
  }
}
