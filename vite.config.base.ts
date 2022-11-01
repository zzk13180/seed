import { defineConfig } from 'vite'
import Vue from '@vitejs/plugin-vue'
import VueJsx from '@vitejs/plugin-vue-jsx'
import windiCSS from 'vite-plugin-windicss'
import angular from '@analogjs/vite-plugin-angular'

// https://vitejs.dev/config/
export const baseconfig = ({ command }) => {
  const isBuild = command === 'build'
  const vueOptions = {
    template: {
      compilerOptions: {
        isCustomElement: (tag: string) => tag.startsWith('my-'),
      },
    },
  }
  const plugins = [VueJsx(), Vue(vueOptions), windiCSS(), angular()]
  const postcssPlugins = []
  const alias = {}
  const proxy = {
    '^/demo/(?=static|theme|sso|api|files)': {
      target: 'https://127.0.0.1:4000',
      configure: (proxy: any) => {
        proxy.on('proxyRes', (proxyRes: any) => {
          const { location } = proxyRes.headers
          console.log(location)
          // if (location) {
          //   proxyRes.headers['location'] = location.replace(/https/, 'http')
          // }
        })
      },
    },
  }
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
    envDir: '../../variables',
    publicDir: '../../public',
    build: {
      outDir: '../../dist/',
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
