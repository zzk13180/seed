import { IsEnum } from 'class-validator'
import { ApiProperty } from '@nestjs/swagger'
import { UserStatus } from '../../../common/enums/user.enum'

/**
 * 更新用户状态 DTO
 */
export class UpdateStatusDto {
  @ApiProperty({
    description: '用户状态',
    enum: UserStatus,
    example: UserStatus.ENABLED,
  })
  @IsEnum(UserStatus, { message: '状态值无效' })
  status!: UserStatus
}
