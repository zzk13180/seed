import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    projects: [
      'packages/libs/*',
      'packages/contracts',
      'packages/kit',
      'packages/db',
      'packages/services',
      'apps/admin',
      'apps/server',
      'apps/api',
    ],
  },
})
