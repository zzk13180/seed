import { IsEnum } from 'class-validator'
import { ApiProperty } from '@nestjs/swagger'
import { UserStatus } from '../../../common/enums/user.enum'
import type { IUpdateStatusDto } from '@seed/api-types'

/**
 * 更新用户状态 DTO
 *
 * 实现 @seed/api-types 中的 IUpdateStatusDto 接口
 */
export class UpdateStatusDto implements IUpdateStatusDto {
  @ApiProperty({
    description: '用户状态',
    enum: UserStatus,
    example: UserStatus.ENABLED,
  })
  @IsEnum(UserStatus, { message: '状态值无效' })
  status!: UserStatus
}
