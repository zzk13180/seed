import { defineConfig } from 'vite'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import windiCSS from 'vite-plugin-windicss'
import { baseconfig } from '../../vite.config.base'

export default defineConfig(async ({ command }) => {
  const baseConfig = await baseconfig({ command })
  const vueOptions = {
    template: {
      compilerOptions: {
        isCustomElement: (tag: string) => tag.startsWith('my-'),
      },
    },
  }
  const plugins = [VueJsx(), Vue(vueOptions), windiCSS()]
  return {
    ...baseConfig,
    base: '/app/',
    plugins,
    build: {
      outDir: './dist/',
      emptyOutDir: true,
      sourcemap: false,
    },
  }
})
