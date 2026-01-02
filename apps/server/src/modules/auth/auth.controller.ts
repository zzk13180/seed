import { Controller, Post, Get, Body, HttpCode, HttpStatus } from '@nestjs/common'
import { ApiTags, ApiOperation, ApiBearerAuth } from '@nestjs/swagger'
import { Public } from '../../common/decorators/public.decorator'
import { GetUser } from '../../common/decorators/get-user.decorator'
import { UserVo } from '../user/vo/user.vo'
import { AuthService } from './auth.service'
import { LoginDto } from './dto/login.dto'
import { LoginVo } from './vo/login.vo'
import type { CurrentUser } from '../../common/interfaces/auth.interface'

/**
 * 认证控制器
 *
 * 注意：响应由 TransformInterceptor 统一包装，Controller 直接返回数据即可
 */
@ApiTags('认证管理')
@Controller('auth')
export class AuthController {
  constructor(private readonly authService: AuthService) {}

  @ApiOperation({ summary: '用户登录' })
  @Public()
  @Post('login')
  @HttpCode(HttpStatus.OK)
  async login(@Body() dto: LoginDto): Promise<LoginVo> {
    return this.authService.login(dto)
  }

  @ApiOperation({ summary: '刷新令牌' })
  @Public()
  @Post('refresh')
  @HttpCode(HttpStatus.OK)
  async refresh(@Body('refreshToken') refreshToken: string): Promise<LoginVo> {
    return this.authService.refreshToken(refreshToken)
  }

  @ApiOperation({ summary: '用户登出' })
  @ApiBearerAuth()
  @Post('logout')
  @HttpCode(HttpStatus.NO_CONTENT)
  async logout(@GetUser() user: CurrentUser): Promise<void> {
    await this.authService.logout(user)
  }

  @ApiOperation({ summary: '获取当前用户信息' })
  @ApiBearerAuth()
  @Get('me')
  async me(@GetUser() user: CurrentUser): Promise<UserVo> {
    return this.authService.getCurrentUser(user)
  }
}
