import Components from 'unplugin-vue-components/vite'
import { CustomElementPlusResolver } from './auto-import-resolver'
import type Path from 'node:path'
import type { Plugin } from 'vite'

/**
 * 排除自动导入的目录
 */
const EXCLUDE_PATTERNS = [/node_modules/, /packages/, /\.git/]

/**
 * 创建 Vue 组件自动导入插件
 *
 * 自动导入 Element Plus 组件和项目内组件
 *
 * @param path Node.js path 模块
 * @returns 组件自动导入插件
 */
export function componentsAutoImportPlugin(path: typeof Path): Plugin {
  const rootDir = path.resolve(import.meta.dirname, '../')
  const dtsPath = path.resolve(rootDir, 'auto-import-components.d.ts')

  return Components({
    resolvers: [CustomElementPlusResolver()],
    dts: dtsPath,
    exclude: EXCLUDE_PATTERNS,
    // 自动导入项目 components 目录下的组件
    dirs: ['src/components'],
    // 组件的有效后缀
    extensions: ['vue'],
    // 深度搜索子目录
    deep: true,
  }) as Plugin
}
