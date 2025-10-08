import { ApiPropertyOptional } from '@nestjs/swagger'
import { IsString, IsEmail, IsOptional, MaxLength, Matches } from 'class-validator'

/**
 * 用户更新 DTO
 */
export class UserUpdateDto {
  @ApiPropertyOptional({ description: '昵称' })
  @IsOptional()
  @IsString()
  @MaxLength(50)
  nickname?: string

  @ApiPropertyOptional({ description: '邮箱' })
  @IsOptional()
  @IsEmail()
  email?: string

  @ApiPropertyOptional({ description: '手机号' })
  @IsOptional()
  @IsString()
  @Matches(/^1[3-9]\d{9}$/, { message: 'Invalid phone number' })
  phone?: string

  @ApiPropertyOptional({ description: '头像URL' })
  @IsOptional()
  @IsString()
  avatar?: string
}
