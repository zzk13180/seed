import path from 'node:path'
import vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import tailwindcss from '@tailwindcss/vite'
import { vitePluginCompression } from './vite-plugin-compression'
import { vitePluginSvgIcons } from './vite-plugin-svg-icons'
import { componentsAutoImportPlugin } from './auto-import-components'
import { VueAutoImportPlugin } from './auto-import-vue'
import type { Plugin, PluginOption } from 'vite'

/**
 * 自定义元素前缀列表
 */
const CUSTOM_ELEMENT_PREFIXES = ['custom-', 'ion-', 'swiper-']

/**
 * 判断标签是否为自定义元素
 */
function isCustomElement(tag: string): boolean {
  return CUSTOM_ELEMENT_PREFIXES.some(prefix => tag.startsWith(prefix))
}

/**
 * 创建 Vite 插件配置
 *
 * @param viteEnv 环境变量
 * @param isBuild 是否为构建模式
 * @returns 插件数组
 */
export function createPlugins(
  viteEnv: Record<string, string>,
  isBuild = false,
): (Plugin | PluginOption)[] {
  const plugins: (Plugin | PluginOption)[] = [
    // Vue 核心插件
    vue({
      template: {
        compilerOptions: {
          isCustomElement,
        },
      },
    }),
    // JSX 支持
    VueJsx(),
    // SVG 图标
    vitePluginSvgIcons(path, isBuild),
    // 组件自动导入
    componentsAutoImportPlugin(path),
    // Vue API 自动导入
    VueAutoImportPlugin(path),
    // Tailwind CSS
    ...tailwindcss(),
    // 压缩插件（仅构建时启用）
    ...vitePluginCompression(viteEnv),
  ]

  return plugins
}
