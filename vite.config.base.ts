import { defineConfig } from 'vite'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import windiCSS from 'vite-plugin-windicss'

// https://vitejs.dev/config/
export const baseconfig = async ({ command }) => {
  const isBuild = command === 'build'
  const plugins = [VueJsx(), Vue(), windiCSS()]
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
      emptyOutDir: true,
      sourcemap: false,
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
