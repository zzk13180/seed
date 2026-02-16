/**
 * @seed/tailwind-config
 *
 * 导出共享的 Tailwind CSS 配置
 */

import seedPreset from './preset.js'

// 导出工具函数
export function withSeedPreset(config) {
  return {
    ...config,
    presets: [...(config.presets || []), seedPreset],
  }
}
export { default as seedPreset } from './preset.js'
