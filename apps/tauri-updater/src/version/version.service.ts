import * as path from 'node:path'
import * as fs from 'fs-extra'
import { Injectable } from '@nestjs/common'
import semverGt from 'semver/functions/gt.js'

export interface UpdateResponse {
  version: string
  pub_date: string
  url: string
  signature: string
  notes: string
}

@Injectable()
export class VersionService {
  private versions: Record<string, UpdateResponse> | null = null
  private readonly dataDir: string
  private readonly versionsFile: string

  constructor() {
    this.dataDir = path.join(process.cwd(), 'data')
    this.versionsFile = path.join(this.dataDir, 'versions.json')
  }

  async getUpdateInfo(target: string, arch: string, currentVersion: string) {
    await this.loadVersions()
    const key = `${target}/${arch}`
    const data = this.versions![key]
    if (!data) return null
    if (!semverGt(data.version, currentVersion)) return null
    return data
  }

  async getAllVersions() {
    return this.loadVersions()
  }

  private async loadVersions() {
    if (this.versions) return this.versions
    try {
      const data = await fs.readJSON(this.versionsFile)
      this.versions = data || {}
    } catch {
      this.versions = {}
    }
    return this.versions
  }
}
