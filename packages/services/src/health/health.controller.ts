import { Hono } from 'hono'
import { HealthService } from './health.service'
import type { AnyDatabase } from '@seed/db'

/**
 * 创建健康检查路由
 *
 * 提供 Kubernetes liveness/readiness probes 标准接口，公开访问无需认证
 *
 * @param db - 数据库实例（用于 readiness 检查）
 */
export function createHealthRoutes(db: AnyDatabase) {
  const healthService = new HealthService(db)

  return (
    new Hono()
      /**
       * GET /liveness — 存活检查
       */
      .get('/liveness', c => {
        return c.json({
          status: 'ok' as const,
          timestamp: new Date().toISOString(),
        })
      })

      /**
       * GET / — 就绪检查
       */
      .get('/', async c => {
        const result = await healthService.checkReadiness()
        return c.json(
          { ...result, timestamp: new Date().toISOString() },
          result.status === 'ok' ? 200 : 503,
        )
      })
  )
}
