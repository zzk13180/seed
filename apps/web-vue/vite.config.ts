import { defineConfig } from 'vite'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import { viteConfig } from '@seed/viteconfig'

export default defineConfig(env => {
  const vueOptions = {
    template: {
      compilerOptions: {
        isCustomElement: (tag: string) => tag.startsWith('custom-'),
      },
    },
  }
  const plugins = [VueJsx(), Vue(vueOptions)]
  return {
    ...viteConfig(env),
    publicDir: 'public',
    envDir: 'env',
    plugins,
    build: {
      outDir: './dist/',
      emptyOutDir: true,
      sourcemap: false,
    },
  }
})
