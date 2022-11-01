import { defineConfig } from 'vite'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import windiCSS from 'vite-plugin-windicss'

export default defineConfig(() => {
  const baseConfig = require('@seed/configs/vite.config.base.cjs')
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
