import * as path from 'node:path'
import * as fs from 'node:fs/promises'
import semverGt from 'semver/functions/gt.js'

/**
 * Tauri 更新响应结构
 */
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
 * 管理桌面应用版本信息和更新检查（内存缓存）
 */
export class TauriUpdaterService {
  private versionsCache: Record<string, UpdateResponse> = {}

  /**
   * 检查是否有新版本
   *
   * @returns 新版本信息或 null（无更新）
   */
  async checkUpdate(
    target: string,
    arch: string,
    currentVersion: string,
  ): Promise<UpdateResponse | null> {
    const versions = await this.loadVersions()
    const key = `${target}/${arch}`
    const data = versions[key]

    if (!data || !semverGt(data.version, currentVersion)) {
      return null
    }

    return data
  }

  /**
   * 获取所有版本信息
   */
  async getAllVersions(): Promise<Record<string, UpdateResponse>> {
    return this.loadVersions()
  }

  private async loadVersions(): Promise<Record<string, UpdateResponse>> {
    if (Object.keys(this.versionsCache).length > 0) {
      return this.versionsCache
    }

    const dataDir =
      process.env.TAURI_UPDATER_DATA_DIR ?? path.join(process.cwd(), 'data/tauri-updater')
    const versionsFile = path.join(dataDir, 'versions.json')

    try {
      const content = await fs.readFile(versionsFile, 'utf-8')
      this.versionsCache = JSON.parse(content) ?? {}
      console.log(
        `Loaded ${Object.keys(this.versionsCache).length} version(s) from ${versionsFile}`,
      )
    } catch (error) {
      console.warn(`Failed to load versions file: ${(error as Error).message}`)
      this.versionsCache = {}
    }

    return this.versionsCache
  }
}
