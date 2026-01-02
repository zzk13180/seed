import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    name: '@seed/common',
    environment: 'node',
    include: ['**/*.test.ts'],
    globals: false,
  },
})
