import { ApiProperty } from '@nestjs/swagger'
import { IsString, MinLength } from 'class-validator'
import type { ILoginDto } from '@seed/api-types'

/**
 * 登录 DTO
 *
 * 实现 @seed/api-types 中的 ILoginDto 接口
 */
export class LoginDto implements ILoginDto {
  @ApiProperty({ description: '用户名', example: 'admin' })
  @IsString()
  username!: string

  @ApiProperty({ description: '密码', example: 'admin123' })
  @IsString()
  @MinLength(6)
  password!: string
}
