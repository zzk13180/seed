import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    projects: [
      'packages/libs/*',
      'packages/contracts',
      'packages/kit',
      'packages/db',
      'packages/services',
      'apps/spa/admin',
      'apps/api/bun',
      'apps/api/edge',
    ],
  },
})
