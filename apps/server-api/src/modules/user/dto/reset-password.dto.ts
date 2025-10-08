import { IsString, MinLength, MaxLength } from 'class-validator'
import { ApiProperty } from '@nestjs/swagger'

/**
 * 重置密码 DTO
 */
export class ResetPasswordDto {
  @ApiProperty({
    description: '新密码',
    minLength: 6,
    maxLength: 32,
    example: 'newPassword123',
  })
  @IsString({ message: '密码必须是字符串' })
  @MinLength(6, { message: '密码长度不能少于 6 个字符' })
  @MaxLength(32, { message: '密码长度不能超过 32 个字符' })
  password!: string
}
