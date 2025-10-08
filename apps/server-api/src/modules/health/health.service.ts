import { Injectable } from '@nestjs/common'
import {
  HealthCheckService,
  TypeOrmHealthIndicator,
  MemoryHealthIndicator,
  DiskHealthIndicator,
} from '@nestjs/terminus'

/**
 * 健康检查服务
 * 提供全面的应用健康状态检测，包括数据库、内存和磁盘
 */
@Injectable()
export class HealthService {
  constructor(
    private readonly health: HealthCheckService,
    private readonly db: TypeOrmHealthIndicator,
    private readonly memory: MemoryHealthIndicator,
    private readonly disk: DiskHealthIndicator,
  ) {}

  /**
   * 简单健康检查 - 用于 Kubernetes liveness probe
   */
  ping() {
    return { status: 'ok', timestamp: new Date().toISOString() }
  }

  /**
   * 全面健康检查 - 用于 Kubernetes readiness probe
   * 检查数据库连接、内存使用和磁盘空间
   */
  async check() {
    return this.health.check([
      // 数据库连接检查
      () => this.db.pingCheck('database'),
      // 堆内存检查 (阈值: 150MB)
      () => this.memory.checkHeap('memory_heap', 150 * 1024 * 1024),
      // RSS 内存检查 (阈值: 300MB)
      () => this.memory.checkRSS('memory_rss', 300 * 1024 * 1024),
      // 磁盘空间检查 (阈值: 90% 使用率)
      () =>
        this.disk.checkStorage('disk', {
          thresholdPercent: 0.9,
          path: '/',
        }),
    ])
  }
}
