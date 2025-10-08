import AutoImport from 'unplugin-auto-import/vite'
import { CustomElementPlusResolver } from './auto-import-resolver'
import type Path from 'node:path'
import type { Plugin } from 'vite'

/**
 * 排除自动导入的目录
 */
const EXCLUDE_PATTERNS = [/node_modules/, /packages/, /\.git/]

/**
 * 创建 Vue API 自动导入插件
 *
 * 自动导入 Vue 的 Composition API（ref, reactive, computed 等）
 *
 * @param path Node.js path 模块
 * @returns 自动导入插件
 */
export function VueAutoImportPlugin(path: typeof Path): Plugin {
  const rootDir = path.resolve(import.meta.dirname, '../')
  const dtsPath = path.resolve(rootDir, 'auto-import-vue.d.ts')

  return AutoImport({
    imports: [
      'vue',
      'vue-router',
      'pinia',
      {
        '@vueuse/core': ['useStorage', 'useDebounceFn', 'useThrottleFn', 'useEventListener'],
      },
    ],
    vueTemplate: true,
    resolvers: [CustomElementPlusResolver()],
    dts: dtsPath,
    exclude: EXCLUDE_PATTERNS,
  }) as Plugin
}
