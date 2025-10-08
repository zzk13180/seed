import { ApiProperty, ApiPropertyOptional } from '@nestjs/swagger'
import { IsString, IsEmail, IsOptional, MinLength, MaxLength, Matches } from 'class-validator'

/**
 * 用户创建 DTO
 */
export class UserCreateDto {
  @ApiProperty({ description: '用户名', example: 'admin' })
  @IsString()
  @MinLength(3)
  @MaxLength(50)
  @Matches(/^\w+$/, {
    message: 'Username can only contain letters, numbers and underscores',
  })
  username!: string

  @ApiProperty({ description: '密码', example: 'admin123' })
  @IsString()
  @MinLength(6)
  @MaxLength(50)
  password!: string

  @ApiPropertyOptional({ description: '昵称', example: 'Admin' })
  @IsOptional()
  @IsString()
  @MaxLength(50)
  nickname?: string

  @ApiPropertyOptional({ description: '邮箱', example: 'admin@example.com' })
  @IsOptional()
  @IsEmail()
  email?: string

  @ApiPropertyOptional({ description: '手机号', example: '13800138000' })
  @IsOptional()
  @IsString()
  @Matches(/^1[3-9]\d{9}$/, { message: 'Invalid phone number' })
  phone?: string

  @ApiPropertyOptional({ description: '头像URL' })
  @IsOptional()
  @IsString()
  avatar?: string
}
