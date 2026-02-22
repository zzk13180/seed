import { z } from 'zod'

const envSchema = z.object({
  NODE_ENV: z.enum(['development', 'production', 'test']).default('development'),
  PORT: z.coerce.number().default(3003),
  API_PREFIX: z.string().default('api'),

  // Database (TCP connection)
  DB_HOST: z.string().default('localhost'),
  DB_PORT: z.coerce.number().default(5432),
  DB_USERNAME: z.string().default('postgres'),
  DB_PASSWORD: z.string().min(1),
  DB_DATABASE: z.string().default('seed'),
  DB_LOGGING: z
    .string()
    .transform(v => v === 'true')
    .default('false'),

  // Better Auth
  BETTER_AUTH_SECRET: z.string().min(1),
  BETTER_AUTH_URL: z.string().url().default('http://localhost:3003'),

  // Redis (optional — falls back to in-memory)
  REDIS_HOST: z.string().default('localhost'),
  REDIS_PORT: z.coerce.number().default(6379),
  REDIS_PASSWORD: z.string().default(''),
  REDIS_DB: z.coerce.number().default(0),

  // Tauri Updater
  TAURI_UPDATER_DATA_DIR: z.string().optional(),
})

export type Env = z.infer<typeof envSchema>

let _env: Env | undefined

export function env(): Env {
  if (!_env) {
    const result = envSchema.safeParse(process.env)
    if (!result.success) {
      console.error('❌ Invalid environment variables:')
      console.error(result.error.flatten().fieldErrors)
      process.exit(1)
    }
    _env = result.data
  }
  return _env
}

export const isDev = () => env().NODE_ENV === 'development'
export const isProd = () => env().NODE_ENV === 'production'
