import { sql } from 'drizzle-orm'
import type { AnyDatabase } from '@seed/db'

interface HealthCheck {
  status: string
  message?: string
}

/**
 * 健康检查服务
 *
 * 封装数据库连通性和运行时资源检查逻辑
 */
export class HealthService {
  constructor(private readonly db: AnyDatabase) {}

  /**
   * 就绪检查 — 检查数据库 + 内存
   */
  async checkReadiness(): Promise<{
    status: 'ok' | 'error'
    details: Record<string, HealthCheck>
  }> {
    const checks: Record<string, HealthCheck> = {}

    // 数据库检查
    try {
      await this.db.execute(sql`SELECT 1`)
      checks.database = { status: 'up' }
    } catch (error) {
      checks.database = {
        status: 'down',
        message: error instanceof Error ? error.message : 'Unknown error',
      }
    }

    // 内存检查（跨运行时兼容：CF Workers 无 process.memoryUsage）
    try {
      const memUsage = process.memoryUsage()
      const heapMB = Math.round(memUsage.heapUsed / 1024 / 1024)
      const rssMB = Math.round(memUsage.rss / 1024 / 1024)

      checks.memory_heap = {
        status: heapMB < 150 ? 'up' : 'down',
        message: `${heapMB}MB used`,
      }
      checks.memory_rss = {
        status: rssMB < 300 ? 'up' : 'down',
        message: `${rssMB}MB used`,
      }
    } catch {
      checks.memory = { status: 'up', message: 'not available in this runtime' }
    }

    const allUp = Object.values(checks).every(c => c.status === 'up')
    return { status: allUp ? 'ok' : 'error', details: checks }
  }
}
