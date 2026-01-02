import { ApiProperty } from '@nestjs/swagger'
import { UserVo } from '../../user/vo/user.vo'

/**
 * 登录响应 VO
 */
export class LoginVo {
  @ApiProperty({ description: '访问令牌' })
  accessToken!: string

  @ApiProperty({ description: '刷新令牌' })
  refreshToken!: string

  @ApiProperty({ description: '用户信息' })
  user!: UserVo

  @ApiProperty({ description: '过期时间（秒）' })
  expiresIn!: number
}
