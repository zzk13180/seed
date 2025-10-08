import compression from 'vite-plugin-compression'
import type { Plugin } from 'vite'

/**
 * 压缩类型
 */
type CompressionType = 'gzip' | 'brotli'

/**
 * 压缩配置
 */
interface CompressionConfig {
  type: CompressionType
  ext: string
  algorithm?: 'brotliCompress'
}

/**
 * 压缩配置映射
 */
const COMPRESSION_CONFIGS: Record<CompressionType, CompressionConfig> = {
  gzip: {
    type: 'gzip',
    ext: '.gz',
  },
  brotli: {
    type: 'brotli',
    ext: '.br',
    algorithm: 'brotliCompress',
  },
}

/**
 * 创建 Vite 压缩插件
 *
 * @param env 环境变量
 * @returns 压缩插件数组
 */
export function vitePluginCompression(env: Record<string, string>): Plugin[] {
  const { VITE_BUILD_COMPRESSION_TYPE } = env

  if (!VITE_BUILD_COMPRESSION_TYPE) {
    return []
  }

  const compressTypes = VITE_BUILD_COMPRESSION_TYPE.split(',')
    .map(type => type.trim().toLowerCase())
    .filter((type): type is CompressionType => type in COMPRESSION_CONFIGS)

  return compressTypes.map(type => {
    const config = COMPRESSION_CONFIGS[type]
    return compression({
      ext: config.ext,
      algorithm: config.algorithm,
      deleteOriginFile: false,
    })
  })
}
