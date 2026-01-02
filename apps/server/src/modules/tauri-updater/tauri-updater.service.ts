import * as path from 'node:path'
import * as fs from 'node:fs/promises'
import { Injectable, Logger } from '@nestjs/common'
import { ConfigService } from '@nestjs/config'
import semverGt from 'semver/functions/gt.js'

export interface UpdateResponse {
  version: string
  pub_date: string
  url: string
  signature: string
  notes: string
}

/**
 * Tauri 更新服务
 *
 * 管理 Tauri 应用的版本信息和更新检查
 */
@Injectable()
export class TauriUpdaterService {
  private readonly logger = new Logger(TauriUpdaterService.name)
  private versions: Record<string, UpdateResponse> = {}
  private readonly versionsFile: string

  constructor(private readonly configService: ConfigService) {
    // 版本配置文件路径，支持通过环境变量配置
    const dataDir = this.configService.get<string>('TAURI_UPDATER_DATA_DIR') || path.join(process.cwd(), 'data/tauri-updater')
    this.versionsFile = path.join(dataDir, 'versions.json')
  }

  /**
   * 检查是否有可用更新
   *
   * @param target - 目标平台
   * @param arch - 架构
   * @param currentVersion - 当前版本
   * @returns 更新信息或 null
   */
  async getUpdateInfo(target: string, arch: string, currentVersion: string): Promise<UpdateResponse | null> {
    await this.loadVersions()
    const key = `${target}/${arch}`
    const data = this.versions[key]

    if (!data) {
      this.logger.debug(`No version found for ${key}`)
      return null
    }

    if (!semverGt(data.version, currentVersion)) {
      this.logger.debug(`Current version ${currentVersion} is up to date (latest: ${data.version})`)
      return null
    }

    this.logger.log(`Update available: ${currentVersion} -> ${data.version} for ${key}`)
    return data
  }

  /**
   * 获取所有版本信息
   */
  async getAllVersions(): Promise<Record<string, UpdateResponse>> {
    return this.loadVersions()
  }

  /**
   * 加载版本配置文件
   */
  private async loadVersions(): Promise<Record<string, UpdateResponse>> {
    if (Object.keys(this.versions).length > 0) return this.versions

    try {
      const content = await fs.readFile(this.versionsFile, 'utf-8')
      this.versions = JSON.parse(content) || {}
      this.logger.log(`Loaded ${Object.keys(this.versions).length} version(s) from ${this.versionsFile}`)
    } catch (error) {
      this.logger.warn(`Failed to load versions file: ${(error as Error).message}`)
      this.versions = {}
    }

    return this.versions
  }

  /**
   * 重新加载版本信息（用于热更新）
   */
  async reloadVersions(): Promise<void> {
    this.versions = {}
    await this.loadVersions()
  }
}
