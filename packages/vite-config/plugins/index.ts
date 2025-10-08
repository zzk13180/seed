import vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import { compression } from 'vite-plugin-compression2'
import { createSvgIconsPlugin as svgIconsPlugin } from 'vite-plugin-svg-icons'
import type { Plugin, PluginOption } from 'vite'
import type { PluginOptions } from '../types'

/**
 * 默认自定义元素前缀
 */
const DEFAULT_CUSTOM_ELEMENT_PREFIXES = ['custom-', 'ion-', 'swiper-']

/**
 * 创建 Vue 相关插件
 */
export function createVuePlugins(options: PluginOptions = {}): PluginOption[] {
  const { enableJsx = true, customElementPrefixes = DEFAULT_CUSTOM_ELEMENT_PREFIXES } = options

  const plugins: PluginOption[] = [
    vue({
      template: {
        compilerOptions: {
          isCustomElement: (tag: string) =>
            customElementPrefixes.some(prefix => tag.startsWith(prefix)),
        },
      },
    }),
  ]

  if (enableJsx) {
    plugins.push(VueJsx())
  }

  return plugins
}

/**
 * 创建压缩插件
 */
export function createCompressionPlugin(
  env: Record<string, string> = {},
  isBuild = false,
): PluginOption[] {
  if (!isBuild) {
    return []
  }

  const plugins: PluginOption[] = []

  // Gzip 压缩
  plugins.push(
    compression({
      algorithm: 'gzip',
      exclude: [/\.(br)$/, /\.(gz)$/],
      threshold: 10240, // 10KB
    }),
  )

  // Brotli 压缩（可选）
  if (env.VITE_BUILD_BROTLI === 'true') {
    plugins.push(
      compression({
        algorithm: 'brotliCompress',
        exclude: [/\.(br)$/, /\.(gz)$/],
        threshold: 10240,
      }),
    )
  }

  return plugins
}

/**
 * 创建 SVG 图标插件
 */
export function createSvgIconsPlugin(iconDirs: string[], isBuild = false): PluginOption {
  return svgIconsPlugin({
    iconDirs,
    symbolId: 'icon-[dir]-[name]',
    svgoOptions: isBuild,
  })
}

/**
 * 创建通用依赖预构建配置
 */
export function createOptimizeDeps(
  additionalIncludes: string[] = [],
  additionalExcludes: string[] = [],
) {
  return {
    include: [
      // Vue 生态
      'vue',
      'vue-router',
      'pinia',
      // 工具库
      'lodash-es',
      '@vueuse/core',
      ...additionalIncludes,
    ],
    exclude: [
      // 排除虚拟模块
      'virtual:svg-icons-register',
      ...additionalExcludes,
    ],
  }
}
