import compression from 'vite-plugin-compression'
import type { Plugin } from 'vite'

export const vitePluginCompression = (environment: Record<string, any>): Plugin[] => {
  const { VITE_BUILD_COMPRESSION_TYPE } = environment
  const plugin: Plugin[] = []
  if (typeof VITE_BUILD_COMPRESSION_TYPE === 'string') {
    const compressList = VITE_BUILD_COMPRESSION_TYPE.split(',')
    if (compressList.includes('gzip')) {
      plugin.push(
        compression({
          ext: '.gz',
          deleteOriginFile: false,
        }),
      )
    }
    if (compressList.includes('brotli')) {
      plugin.push(
        compression({
          ext: '.br',
          algorithm: 'brotliCompress',
          deleteOriginFile: false,
        }),
      )
    }
  }
  return plugin
}
