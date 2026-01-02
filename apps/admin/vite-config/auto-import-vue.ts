import AutoImport from 'unplugin-auto-import/vite'
import { ElementPlusResolver } from 'unplugin-vue-components/resolvers'
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
  // ESLint 全局变量配置文件路径（由 unplugin-auto-import 自动生成）
  const eslintrcPath = path.resolve(rootDir, '.eslintrc-auto-import.json')

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
    resolvers: [ElementPlusResolver()],
    dts: dtsPath,
    exclude: EXCLUDE_PATTERNS,
    /**
     * 生成 ESLint 全局变量配置
     * 该文件包含所有自动导入的 API，供 ESLint 识别为全局变量
     * 避免在 eslint.config.js 中手动维护 vue3Globals/elementPlusGlobals
     */
    eslintrc: {
      enabled: true,
      filepath: eslintrcPath,
      globalsPropValue: true, // 设置为 true 表示这些是只读全局变量
    },
  }) as Plugin
}
