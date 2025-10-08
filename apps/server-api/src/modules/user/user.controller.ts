import {
  Controller,
  Get,
  Post,
  Put,
  Delete,
  Patch,
  Body,
  Param,
  Query,
  ParseIntPipe,
  HttpCode,
  HttpStatus,
} from '@nestjs/common'
import { ApiTags, ApiOperation, ApiBearerAuth } from '@nestjs/swagger'
import { PageResultDto } from '../../common/dto/page-result.dto'
import { Permissions } from '../../common/decorators/permissions.decorator'
import { Roles } from '../../common/decorators/roles.decorator'
import { Permission, Role } from '../../common/enums/user.enum'
import { UserService } from './user.service'
import { UserCreateDto } from './dto/user-create.dto'
import { UserUpdateDto } from './dto/user-update.dto'
import { UserQueryDto } from './dto/user-query.dto'
import { BatchDeleteDto } from './dto/batch-delete.dto'
import { UpdateStatusDto } from './dto/update-status.dto'
import { ResetPasswordDto } from './dto/reset-password.dto'
import { UserVo } from './vo/user.vo'

/**
 * 用户管理控制器
 *
 * 注意：响应由 TransformInterceptor 统一包装，Controller 直接返回数据即可
 */
@ApiTags('用户管理')
@ApiBearerAuth()
@Controller('v1/users')
export class UserController {
  constructor(private readonly userService: UserService) {}

  @ApiOperation({ summary: '分页查询用户' })
  @Get()
  @Permissions(Permission.USER_READ)
  async page(@Query() query: UserQueryDto): Promise<PageResultDto<UserVo>> {
    return this.userService.page(query)
  }

  @ApiOperation({ summary: '查询所有用户' })
  @Get('list')
  @Permissions(Permission.USER_READ)
  async list(): Promise<UserVo[]> {
    return this.userService.findAll()
  }

  @ApiOperation({ summary: '根据ID查询用户' })
  @Get(':id')
  @Permissions(Permission.USER_READ)
  async getById(@Param('id', ParseIntPipe) id: number): Promise<UserVo> {
    return this.userService.findById(id)
  }

  @ApiOperation({ summary: '创建用户' })
  @Post()
  @Permissions(Permission.USER_WRITE)
  async create(@Body() dto: UserCreateDto): Promise<UserVo> {
    return this.userService.create(dto)
  }

  @ApiOperation({ summary: '更新用户' })
  @Put(':id')
  @Permissions(Permission.USER_WRITE)
  async update(@Param('id', ParseIntPipe) id: number, @Body() dto: UserUpdateDto): Promise<UserVo> {
    return this.userService.update(id, dto)
  }

  @ApiOperation({ summary: '删除用户' })
  @Delete(':id')
  @Permissions(Permission.USER_DELETE)
  @Roles(Role.ADMIN)
  @HttpCode(HttpStatus.NO_CONTENT)
  async delete(@Param('id', ParseIntPipe) id: number): Promise<void> {
    await this.userService.delete(id)
  }

  @ApiOperation({ summary: '批量删除用户' })
  @Post('batch-delete')
  @Roles(Role.ADMIN)
  @HttpCode(HttpStatus.NO_CONTENT)
  async deleteBatch(@Body() dto: BatchDeleteDto): Promise<void> {
    await this.userService.deleteBatch(dto.ids)
  }

  @ApiOperation({ summary: '更新用户状态' })
  @Patch(':id/status')
  @Roles(Role.ADMIN)
  @HttpCode(HttpStatus.NO_CONTENT)
  async updateStatus(
    @Param('id', ParseIntPipe) id: number,
    @Body() dto: UpdateStatusDto,
  ): Promise<void> {
    await this.userService.updateStatus(id, dto.status)
  }

  @ApiOperation({ summary: '重置用户密码' })
  @Patch(':id/password')
  @Roles(Role.ADMIN)
  @HttpCode(HttpStatus.NO_CONTENT)
  async resetPassword(
    @Param('id', ParseIntPipe) id: number,
    @Body() dto: ResetPasswordDto,
  ): Promise<void> {
    await this.userService.resetPassword(id, dto.password)
  }
}
