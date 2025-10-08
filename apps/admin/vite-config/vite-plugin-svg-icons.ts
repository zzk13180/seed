import { createSvgIconsPlugin } from 'vite-plugin-svg-icons'
import type * as Path from 'node:path'
import type { Plugin } from 'vite'

/**
 * SVG 图标目录名
 */
const SVG_ICONS_DIR = 'assets/svg-icons'

/**
 * SVG symbolId 格式
 * - icon-[dir]-[name]: 按目录和文件名生成
 */
const SYMBOL_ID_FORMAT = 'icon-[dir]-[name]'

/**
 * 创建 SVG 图标插件
 *
 * @param path Node.js path 模块
 * @param isBuild 是否为构建模式
 * @returns SVG 图标插件
 */
export function vitePluginSvgIcons(path: typeof Path, isBuild: boolean): Plugin {
  const srcDir = path.resolve(import.meta.dirname, '../src')
  const iconDir = path.resolve(srcDir, SVG_ICONS_DIR)

  return createSvgIconsPlugin({
    // 指定需要缓存的图标文件夹
    iconDirs: [iconDir],
    // 指定 symbolId 格式
    symbolId: SYMBOL_ID_FORMAT,
    // 构建模式下启用 SVGO 优化
    svgoOptions: isBuild,
  })
}
