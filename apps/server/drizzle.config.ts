import { join } from 'node:path'
import { defineConfig } from 'drizzle-kit'
import * as dotenv from 'dotenv'

// 根据环境加载对应的 .env 文件
const envFile = process.env.NODE_ENV === 'production' ? '.env.production' : '.env.development'
dotenv.config({ path: join(__dirname, envFile) })

/**
 * Drizzle Kit 配置
 *
 * 用于数据库迁移命令：
 * - pnpm drizzle-kit generate  生成迁移文件
 * - pnpm drizzle-kit migrate   执行迁移
 * - pnpm drizzle-kit push      推送 schema 到数据库（开发用）
 * - pnpm drizzle-kit studio    打开 Drizzle Studio
 */
export default defineConfig({
  dialect: 'postgresql',
  schema: './src/common/database/schema/index.ts',
  out: './src/migrations',

  dbCredentials: {
    host: process.env.DB_HOST || 'localhost',
    port: Number.parseInt(process.env.DB_PORT || '5432', 10),
    user: process.env.DB_USERNAME || 'postgres',
    password: process.env.DB_PASSWORD || 'password',
    database: process.env.DB_DATABASE || 'seed',
  },

  // 迁移表名
  migrations: {
    table: 'drizzle_migrations',
  },

  // 启用详细输出
  verbose: true,
  // 严格模式
  strict: true,
})
