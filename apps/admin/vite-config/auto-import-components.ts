import Components from 'unplugin-vue-components/vite'
import { CustomElementPlusResolver } from './auto-import-resolver'
import type Path from 'node:path'
import type { Plugin } from 'vite'
import type { ComponentResolver } from 'unplugin-vue-components'

/**
 * 排除自动导入的目录
 */
const EXCLUDE_PATTERNS = [/node_modules/, /packages/, /\.git/]

/**
 * Lucide 图标自动导入解析器
 *
 * 支持以下使用方式:
 * - <LucideHome /> 或 <lucide-home />
 * - <IconHome /> 或 <icon-home />
 */
function LucideIconResolver(): ComponentResolver {
  return {
    type: 'component',
    resolve: (name: string) => {
      // 匹配 LucideXxx 或 IconXxx 格式
      let iconName: string | null = null

      if (name.startsWith('Lucide')) {
        iconName = name.slice(6) // 移除 'Lucide' 前缀
      } else if (name.startsWith('Icon') && name.length > 4) {
        iconName = name.slice(4) // 移除 'Icon' 前缀
      }

      if (iconName) {
        return {
          name: iconName,
          from: 'lucide-vue-next',
        }
      }
    },
  }
}

/**
 * 创建 Vue 组件自动导入插件
 *
 * 自动导入 Element Plus 组件、Lucide 图标和项目内组件
 *
 * @param path Node.js path 模块
 * @returns 组件自动导入插件
 */
export function componentsAutoImportPlugin(path: typeof Path): Plugin {
  const rootDir = path.resolve(import.meta.dirname, '../')
  const dtsPath = path.resolve(rootDir, 'auto-import-components.d.ts')

  return Components({
    resolvers: [CustomElementPlusResolver(), LucideIconResolver()],
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
