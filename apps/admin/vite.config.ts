import path from 'node:path'
import { defineConfig, loadEnv } from 'vite'
import { createPlugins } from './vite-config/createPlugins'
import { optimizeDeps } from './vite-config/optimization'

import type { UserConfig, ConfigEnv } from 'vite'

export default defineConfig(({ mode, command }: ConfigEnv): UserConfig => {
  const env = loadEnv(mode, process.cwd())

  // Hono API 后端地址
  const apiTarget = env.VITE_API_URL

  const proxy = {
    '/api': {
      target: apiTarget,
      changeOrigin: true,
      ws: true,
      // Hono 服务端已在 /api 下注册路由，无需 rewrite
    },
  }
  return {
    base: '/',
    resolve: {
      alias: {
        '@': path.resolve(import.meta.dirname, './src'),
      },
    },
    server: {
      proxy,
      // headers: {
      //   'Cross-Origin-Opener-Policy': 'same-origin',
      //   'Cross-Origin-Embedder-Policy': 'require-corp'
      // },
      // allowedHosts: ['zzk13180.local'],
      host: '0.0.0.0',
    },
    preview: {
      proxy,
    },
    css: {
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
    plugins: createPlugins(env, command === 'build'),
    optimizeDeps,
  }
})
