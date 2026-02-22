import path from 'node:path'
import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    name: '@seed/spa-admin',
    globals: true,
    root: path.resolve(import.meta.dirname),
    include: ['src/**/*.test.ts', 'src/**/*.spec.ts'],
    environment: 'node',
    alias: {
      '@': path.resolve(import.meta.dirname, './src'),
      '@seed/contracts': path.resolve(import.meta.dirname, '../../packages/contracts/src'),
    },
  },
})
