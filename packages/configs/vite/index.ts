/**
 * @seed/vite-config - 共享 Vite 配置工具包
 *
 * 提供统一的 Vite 配置、插件和工具函数
 */

export { createBaseConfig, mergeConfig } from './utils/index'
export { createVuePlugins, createCompressionPlugin, createSvgIconsPlugin } from './plugins/index'
export type { ViteConfigOptions, PluginOptions } from './types'
