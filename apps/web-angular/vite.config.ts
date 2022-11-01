import { defineConfig } from 'vite'
import angular from '@analogjs/vite-plugin-angular'

export default defineConfig(() => {
  const baseConfig = require('@seed/configs/vite.config.base.cjs')
  const plugins = [angular()]
  return {
    ...baseConfig,
    plugins,
    root: 'src',
    publicDir: 'assets',
    build: {
      outDir: './dist/',
      emptyOutDir: true,
      target: 'es2020',
    },
    resolve: {
      mainFields: ['module'],
    },
  }
})
