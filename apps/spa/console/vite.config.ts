import path from 'node:path'
import { defineConfig, loadEnv } from 'vite'
import vue from '@vitejs/plugin-vue'

import type { UserConfig, ConfigEnv } from 'vite'

export default defineConfig(({ mode }: ConfigEnv): UserConfig => {
  const env = loadEnv(mode, process.cwd())

  return {
    base: '/',
    resolve: {
      alias: {
        '@': path.resolve(import.meta.dirname, './src'),
      },
    },
    server: {
      host: '0.0.0.0',
      port: 8081,
      proxy: {
        '/api': {
          // eslint-disable-next-line sonarjs/no-clear-text-protocols
          target: env.VITE_API_URL || 'http://u20:5002',
          changeOrigin: true,
        },
      },
    },
    css: {
      preprocessorOptions: {
        scss: {
          // @ts-ignore
          javascriptEnabled: true,
        },
      },
    },
    plugins: [
      vue({
        template: {
          compilerOptions: {
            isCustomElement: tag =>
              ['custom-', 'ion-', 'swiper-'].some(p => tag.startsWith(p)),
          },
        },
      }),
    ],
  }
})
