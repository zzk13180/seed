import { defineConfig } from 'vite'
import angular from '@analogjs/vite-plugin-angular'
import { baseconfig } from '../../vite.config.base'

export default defineConfig(({ command }) => {
  const baseConfig = baseconfig({ command })
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
