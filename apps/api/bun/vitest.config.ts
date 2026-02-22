import path from 'node:path'
import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    name: '@seed/server',
    globals: true,
    root: path.resolve(import.meta.dirname),
    include: ['src/**/*.spec.ts', 'src/**/*.test.ts'],
    environment: 'node',
    alias: {
      '@': path.resolve(import.meta.dirname, './src'),
    },
  },
})
