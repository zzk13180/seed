import { env } from './config/env.config'
import { createApp } from './app'
import { getPool } from './database/database.provider'

const { app } = createApp()
const port = env().PORT

console.log(`🚀 Seed Server running on http://localhost:${port}`)
console.log(`📚 Auth endpoints: http://localhost:${port}/api/auth`)
console.log(`❤️  Health check: http://localhost:${port}/api/health`)

// Graceful shutdown
const shutdown = async () => {
  console.log('Shutting down...')
  const pool = getPool()
  if (pool) {
    await pool.end()
  }
  process.exit(0)
}

process.on('SIGTERM', shutdown)
process.on('SIGINT', shutdown)

export default {
  port,
  fetch: app.fetch,
}
