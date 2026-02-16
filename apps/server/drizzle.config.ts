import { join } from 'node:path'
import { defineConfig } from 'drizzle-kit'
import * as dotenv from 'dotenv'

const envFile = process.env.NODE_ENV === 'production' ? '.env.production' : '.env.development'
dotenv.config({ path: join(import.meta.dirname, envFile) })

export default defineConfig({
  dialect: 'postgresql',
  schema: '../../packages/db/src/schema/index.ts',
  out: './drizzle',

  dbCredentials: {
    host: process.env.DB_HOST || 'localhost',
    port: Number.parseInt(process.env.DB_PORT || '5432', 10),
    user: process.env.DB_USERNAME || 'postgres',
    password: process.env.DB_PASSWORD || 'password',
    database: process.env.DB_DATABASE || 'seed',
    ssl: false,
  },

  verbose: true,
  strict: true,
})
