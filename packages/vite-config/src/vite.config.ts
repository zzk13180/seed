import { colorPalette, genLessVars } from './theme'
import type { UserConfigFn } from 'vite'

export const viteConfig: UserConfigFn = ({ command }) => {
  const isBuild = command === 'build'
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
  return {
    server: {
      host: '0.0.0.0', // listen on all addresses
      port: 8080,
      proxy,
    },
    resolve: { alias },
    clearScreen: false,
    optimizeDeps: {
      include,
      exclude: ['vue-demi'],
    },
    css: {
      postcss: {
        plugins: postcssPlugins,
      },
      preprocessorOptions: {
        less: {
          globalVars: genLessVars(),
          javascriptEnabled: true,
          functions: {
            colorPalette,
          },
        },
      },
    },
    define: {
      __APP_VERSION__: JSON.stringify('v0.0.1'),
    },
  }
}
