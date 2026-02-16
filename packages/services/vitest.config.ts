import path from 'node:path'
import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    name: '@seed/services',
    globals: true,
    root: path.resolve(import.meta.dirname),
    include: ['src/**/*.test.ts', 'src/**/*.spec.ts'],
    environment: 'node',
  },
})
