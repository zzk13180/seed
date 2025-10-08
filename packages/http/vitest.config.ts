import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    name: '@seed/http',
    environment: 'node',
    include: ['**/*.test.ts'],
    globals: false,
  },
})
