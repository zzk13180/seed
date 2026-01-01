import { Controller, Get, Param, Res, HttpStatus, Query } from '@nestjs/common'
import { VersionService } from './version.service'
import type { FastifyReply } from 'fastify'

@Controller()
export class VersionController {
  constructor(private readonly versionService: VersionService) {}

  // GET /appimage/:arch/:currentVersion - Tauri 更新检查
  @Get(':target/:arch/:currentVersion')
  async getUpdate(
    @Param('target') target: string,
    @Param('arch') arch: string,
    @Param('currentVersion') currentVersion: string,
    @Res() res: FastifyReply,
  ) {
    try {
      const result = await this.versionService.getUpdateInfo(target, arch, currentVersion)
      if (!result) {
        res.status(HttpStatus.NO_CONTENT).send()
        return
      }
      res.status(HttpStatus.OK).send(result)
    } catch {
      res.status(HttpStatus.NO_CONTENT).send()
    }
  }

  @Get('')
  getAllVersions() {
    return this.versionService.getAllVersions()
  }
}
