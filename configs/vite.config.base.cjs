module.exports = ({ command }) => {
  const isBuild = command === 'build'
  const postcssPlugins = []
  const alias = {}
  const proxy = {
    '^/demo/(?=static|theme|sso|api|files)': {
      target: 'https://127.0.0.1:4000',
      configure: (proxy) => {
        proxy.on('proxyRes', (proxyRes) => {
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
  }
}
