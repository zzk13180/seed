import path from 'node:path'
import type { UserConfig } from 'vite'
import type { ViteConfigOptions } from '../types'

/**
 * 创建基础 Vite 配置
 */
export function createBaseConfig(options: ViteConfigOptions = {}): UserConfig {
  const { root = process.cwd(), base = '/', alias = {}, port = 5173, proxy = {} } = options

  return {
    base,
    resolve: {
      alias: {
        '@': path.resolve(root, './src'),
        ...alias,
      },
    },
    server: {
      host: '0.0.0.0',
      port,
      proxy: (() => {
        const resolvedProxy: Record<string, any> = {}
        for (const [key, value] of Object.entries(proxy)) {
          resolvedProxy[key] = {
            target: value.target,
            changeOrigin: value.changeOrigin ?? true,
            ws: value.ws ?? false,
            rewrite: value.rewrite,
          }
        }
        return resolvedProxy
      })(),
    },
    css: {
      postcss: {
        plugins: [
          {
            postcssPlugin: 'internal:charset-removal',
            AtRule: {
              charset: (atRule: any) => {
                if (atRule.name === 'charset') {
                  atRule.remove()
                }
              },
            },
          },
        ],
      },
    },
    build: {
      target: 'esnext',
      cssTarget: 'chrome80',
      reportCompressedSize: false,
      chunkSizeWarningLimit: 2000,
      rollupOptions: {
        output: {
          manualChunks: {
            vue: ['vue', 'vue-router', 'pinia'],
          },
        },
      },
    },
  }
}

/**
 * 深度合并配置
 */
export function mergeConfig(baseConfig: UserConfig, overrides: Partial<UserConfig>): UserConfig {
  return {
    ...baseConfig,
    ...overrides,
    resolve: {
      ...baseConfig.resolve,
      ...overrides.resolve,
      alias: {
        ...(baseConfig.resolve?.alias as Record<string, string>),
        ...(overrides.resolve?.alias as Record<string, string>),
      },
    },
    server: {
      ...baseConfig.server,
      ...overrides.server,
      proxy: {
        ...baseConfig.server?.proxy,
        ...overrides.server?.proxy,
      },
    },
    build: {
      ...baseConfig.build,
      ...overrides.build,
    },
  }
}
