import { createSvgIconsPlugin } from 'vite-plugin-svg-icons'
import type * as Path from 'path'

export const vitePluginSvgIcons = (path: typeof Path, isBuild: boolean) => {
  const iconDir: string = path.resolve(
    path.resolve(import.meta.dirname, '../src'),
    'assets/svg-icons',
  )
  return createSvgIconsPlugin({
    // 指定需要缓存的图标文件夹
    iconDirs: [iconDir],
    // 指定symbolId格式
    symbolId: 'icon-[dir]-[name]',
    svgoOptions: isBuild,
  })
}
