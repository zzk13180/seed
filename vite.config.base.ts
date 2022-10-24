import { defineConfig } from 'vite'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import windiCSS from 'vite-plugin-windicss'

// https://vitejs.dev/config/
export const baseconfig = async ({ command }) => {
  const isBuild = command === 'build'
  const vueOptions = {
    template: {
      compilerOptions: {
        isCustomElement: (tag: string) => tag.startsWith('my-'),
      },
    },
  }
  const plugins = [VueJsx(), Vue(vueOptions), windiCSS()]
  const postcssPlugins = []
  const alias = {}
  const proxy = {}
  const include = []
  return defineConfig({
    plugins,
    server: {
      host: '127.0.0.1',
      port: 8080,
      proxy,
    },
    resolve: { alias },
    clearScreen: false,
    envDir: '../../',
    publicDir: '../../public',
    build: {
      outDir: '../../out/',
      emptyOutDir: false,
      sourcemap: true,
      rollupOptions: {
        external: /^lit/,
      },
    },
    optimizeDeps: {
      include,
      exclude: ['vue-demi'],
    },
    css: {
      postcss: {
        plugins: postcssPlugins,
      },
      preprocessorOptions: {},
    },
    define: {
      __DEV__: !isBuild,
    },
  })
}
