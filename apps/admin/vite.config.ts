import path from 'node:path'
import { defineConfig, loadEnv } from 'vite'
import { createPlugins } from './vite-config/createPlugins'
import { optimizeDeps } from './vite-config/optimization'

import type { UserConfig, ConfigEnv } from 'vite'

export default defineConfig(({ mode, command }: ConfigEnv): UserConfig => {
  const env = loadEnv(mode, process.cwd())

  const proxy = {
    [env.VITE_API_BASE_PATH]: {
      target: 'http://127.0.0.1:3000',
      changeOrigin: true,
      ws: true,

      rewrite: (path: string) =>
        // eslint-disable-next-line security/detect-non-literal-regexp
        path.replace(new RegExp(`^${env.VITE_API_BASE_PATH}`), ''),
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
