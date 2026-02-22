import { defineConfig } from 'drizzle-kit'

export default defineConfig({
  dialect: 'postgresql',
  schema: '../../packages/db/src/schema/*.ts',
  out: '../../packages/db/drizzle',
  dbCredentials: {
    url: process.env.DATABASE_URL!,
  },
  migrations: {
    table: 'drizzle_migrations',
  },
  verbose: true,
  strict: true,
})
