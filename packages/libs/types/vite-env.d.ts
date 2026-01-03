interface ImportMetaEnv {
  readonly MODE: string // 当前模式
  readonly BASE_URL: string // 应用基础 URL
  readonly BASE_PATH?: string // 自定义基础路径
  readonly PROD: boolean // 是否生产环境
  readonly DEV: boolean // 是否开发环境
  readonly SSR: boolean // 是否服务端渲染
  readonly VITE_API_BASE_PATH: string // API 基础路径
  readonly VITE_BUILD_COMPRESSION_TYPE?: 'gzip' | 'brotli' // 打包压缩类型
}

interface ImportMeta {
  readonly env: ImportMetaEnv
  readonly hot?: import('vite/types/hot.js').ViteHotContext
}
