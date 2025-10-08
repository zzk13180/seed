import path from 'node:path'
import { defineConfig, loadEnv } from 'vite'
import { createPlugins } from './vite/plugins'
import { optimizeDeps } from './vite/optimization'

import type { UserConfig, ConfigEnv } from 'vite'

export default defineConfig(({ mode, command }: ConfigEnv): UserConfig => {
  const environment = loadEnv(mode, process.cwd())

  return {
    base: '/',
    resolve: {
      alias: {
        '@': path.resolve(import.meta.dirname, './src'),
      },
    },
    server: {
      // headers: {
      //   'Cross-Origin-Opener-Policy': 'same-origin',
      //   'Cross-Origin-Embedder-Policy': 'require-corp'
      // },
      host: '0.0.0.0',
      port: 8081,
      proxy: {
        '/api': {
          // eslint-disable-next-line sonarjs/no-clear-text-protocols
          target: 'http://u20:5002',
          changeOrigin: true,
        },
      },
    },
    preview: {},
    css: {
      preprocessorOptions: {
        scss: {
          // @ts-ignore
          javascriptEnabled: true,
        },
      },
      postcss: {
        plugins: [
          {
            postcssPlugin: 'internal:charset-removal',
            AtRule: {
              charset: atRule => {
                if (atRule.name === 'charset') {
                  atRule.remove()
                }
              },
            },
          },
        ],
      },
    },
    plugins: createPlugins(environment, command === 'build'),
    optimizeDeps,
  }
})
