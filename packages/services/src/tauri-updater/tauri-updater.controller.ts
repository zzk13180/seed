import { Hono } from 'hono'
import { zValidator } from '@hono/zod-validator'
import { z } from 'zod'
import { TauriUpdaterService } from './tauri-updater.service'

const checkParamsSchema = z.object({
  target: z.string(),
  arch: z.string(),
  currentVersion: z.string(),
})

/**
 * 创建 Tauri 更新路由
 *
 * Tauri updater 标准接口，公开访问无需认证
 */
export function createTauriUpdaterRoutes() {
  const updaterService = new TauriUpdaterService()

  return (
    new Hono()
      /**
       * GET /:target/:arch/:currentVersion — 检查更新
       */
      .get('/:target/:arch/:currentVersion', zValidator('param', checkParamsSchema), async c => {
        const { target, arch, currentVersion } = c.req.valid('param')
        const update = await updaterService.checkUpdate(target, arch, currentVersion)

        if (!update) {
          return c.body(null, 204)
        }

        return c.json(update)
      })

      /**
       * GET /versions — 获取所有版本信息
       */
      .get('/versions', async c => {
        const versions = await updaterService.getAllVersions()
        return c.json(versions)
      })
  )
}
