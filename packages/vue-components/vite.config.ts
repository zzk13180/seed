import { defineConfig } from 'vite'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import { viteConfig } from '@seed/vite-config'

export default defineConfig(env => {
  const plugins = [VueJsx(), Vue()]
  return {
    ...viteConfig(env),
    publicDir: 'public',
    envDir: 'env',
    plugins,
    build: {
      outDir: './dist/',
      emptyOutDir: true,
      sourcemap: false,
      lib: {
        entry: 'src/index.ts',
        name: 'vue-components',
      },
      rollupOptions: {
        external: ['vue'],
        output: {
          globals: {
            vue: 'Vue',
          },
        },
      },
    },
  }
})
