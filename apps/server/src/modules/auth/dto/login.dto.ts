import { ApiProperty } from '@nestjs/swagger'
import { IsString, MinLength } from 'class-validator'

/**
 * 登录 DTO
 */
export class LoginDto {
  @ApiProperty({ description: '用户名', example: 'admin' })
  @IsString()
  username!: string

  @ApiProperty({ description: '密码', example: 'admin123' })
  @IsString()
  @MinLength(6)
  password!: string
}
