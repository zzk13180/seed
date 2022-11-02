import { defineConfig } from 'vite'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import windiCSS from 'vite-plugin-windicss'
import { viteConfig, windiConfig } from '@seed/configs'

export default defineConfig(env => {
  const vueOptions = {
    template: {
      compilerOptions: {
        isCustomElement: (tag: string) => tag.startsWith('my-'),
      },
    },
  }
  const plugins = [VueJsx(), Vue(vueOptions), windiCSS(windiConfig)]
  return {
    ...viteConfig(env),
    root: 'src',
    publicDir: 'assets',
    plugins,
    build: {
      outDir: './dist/',
      emptyOutDir: true,
      sourcemap: false,
    },
  }
})
