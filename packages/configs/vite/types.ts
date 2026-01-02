import type { UserConfig, PluginOption } from 'vite'

/**
 * Vite 配置选项
 */
export interface ViteConfigOptions {
  /** 应用根目录 */
  root?: string
  /** 基础路径 */
  base?: string
  /** 路径别名 */
  alias?: Record<string, string>
  /** 服务器端口 */
  port?: number
  /** 代理配置 */
  proxy?: Record<string, ProxyOptions>
  /** 是否为构建模式 */
  isBuild?: boolean
}

/**
 * 代理配置选项
 */
export interface ProxyOptions {
  target: string
  changeOrigin?: boolean
  ws?: boolean
  rewrite?: (path: string) => string
}

/**
 * 插件配置选项
 */
export interface PluginOptions {
  /** 是否启用 Vue JSX */
  enableJsx?: boolean
  /** 是否启用压缩 */
  enableCompression?: boolean
  /** 是否启用 SVG 图标 */
  enableSvgIcons?: boolean
  /** SVG 图标目录 */
  svgIconsDir?: string
  /** 自定义元素前缀 */
  customElementPrefixes?: string[]
}

/**
 * 依赖优化配置
 */
export interface OptimizeDepsOptions {
  /** 强制预构建的依赖 */
  include?: string[]
  /** 排除预构建的依赖 */
  exclude?: string[]
}
