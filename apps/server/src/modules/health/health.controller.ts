import { Controller, Get } from '@nestjs/common'
import { ApiTags, ApiOperation } from '@nestjs/swagger'
import { HealthCheck } from '@nestjs/terminus'
import { Public } from '../../common/decorators/public.decorator'
import { HealthService } from './health.service'

@ApiTags('Health')
@Controller('health')
export class HealthController {
  constructor(private readonly healthService: HealthService) {}

  /**
   * 简单存活检查 - Kubernetes liveness probe
   * 只返回服务是否在运行，不检查依赖
   */
  @Get('liveness')
  @Public()
  @ApiOperation({ summary: 'Liveness probe - 检查服务是否存活' })
  liveness() {
    return this.healthService.ping()
  }

  /**
   * 就绪检查 - Kubernetes readiness probe
   * 检查所有依赖项（数据库、内存、磁盘）
   */
  @Get()
  @Public()
  @HealthCheck()
  @ApiOperation({ summary: 'Readiness probe - 检查服务及依赖是否就绪' })
  check() {
    return this.healthService.check()
  }
}
