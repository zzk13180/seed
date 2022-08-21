import { defineConfig } from 'vite'
import { baseconfig } from '../../vite.config.base'

export default defineConfig(async ({ command }) => {
  const baseConfig = await baseconfig({ command })
  return {
    ...baseConfig,
    base: '/app/',
    build: {
      outDir: './app/',
      emptyOutDir: true,
      sourcemap: false,
    },
  }
})
