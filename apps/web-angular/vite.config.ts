import { defineConfig } from 'vite'
import { baseconfig } from '../../vite.config.base'

export default defineConfig(({ command }) => {
  const baseConfig = baseconfig({ command })
  return {
    ...baseConfig,
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
