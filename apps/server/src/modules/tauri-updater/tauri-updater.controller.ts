import { Controller, Get, Param, Res, HttpStatus } from '@nestjs/common'
import { TauriUpdaterService } from './tauri-updater.service'
import type { Response } from 'express'

/**
 * Tauri 应用更新控制器
 *
 * 提供 Tauri 应用的版本检查和更新服务
 * 路由前缀: /updater
 */
@Controller('updater')
export class TauriUpdaterController {
  constructor(private readonly updaterService: TauriUpdaterService) {}

  /**
   * 检查更新 - Tauri updater 标准接口
   * GET /api/updater/:target/:arch/:currentVersion
   *
   * @param target - 目标平台 (linux, darwin, windows)
   * @param arch - 架构 (x86_64, aarch64)
   * @param currentVersion - 当前版本号
   * @returns 更新信息或 204 无更新
   */
  @Get(':target/:arch/:currentVersion')
  async checkUpdate(
    @Param('target') target: string,
    @Param('arch') arch: string,
    @Param('currentVersion') currentVersion: string,
    @Res() res: Response,
  ) {
    try {
      const result = await this.updaterService.getUpdateInfo(target, arch, currentVersion)
      if (!result) {
        res.status(HttpStatus.NO_CONTENT).send()
        return
      }
      res.status(HttpStatus.OK).json(result)
    } catch {
      res.status(HttpStatus.NO_CONTENT).send()
    }
  }

  /**
   * 获取所有版本信息
   * GET /api/updater/versions
   */
  @Get('versions')
  getAllVersions() {
    return this.updaterService.getAllVersions()
  }
}
