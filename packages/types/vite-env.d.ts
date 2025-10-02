interface ImportMetaEnv {
  readonly VITE_API_BASE_PATH: string // API 基础路径
  readonly VITE_BUILD_COMPRESSION_TYPE?: 'gzip' | 'brotli' // 打包压缩类型
}

interface ImportMeta {
  readonly env: ImportMetaEnv
}
