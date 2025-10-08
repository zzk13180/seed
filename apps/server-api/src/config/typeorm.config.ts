import { join } from 'node:path'
import { DataSource } from 'typeorm'
import * as dotenv from 'dotenv'

// 根据环境加载对应的 .env 文件
const envFile = process.env.NODE_ENV === 'production' ? '.env.production' : '.env.development'

dotenv.config({ path: join(__dirname, '../../', envFile) })

/**
 * TypeORM CLI 数据源配置
 *
 * 用于数据库迁移命令：
 * - pnpm migration:generate -- -n MigrationName
 * - pnpm migration:run
 * - pnpm migration:revert
 */
export default new DataSource({
  type: 'mysql',
  host: process.env.DB_HOST || 'localhost',
  port: Number.parseInt(process.env.DB_PORT || '3306', 10),
  username: process.env.DB_USERNAME || 'root',
  password: process.env.DB_PASSWORD || 'password',
  database: process.env.DB_DATABASE || 'seed',

  // Entity 路径
  entities: [join(__dirname, '../**/*.entity{.ts,.js}')],

  // Migration 配置
  migrations: [join(__dirname, '../migrations/*{.ts,.js}')],
  migrationsTableName: 'typeorm_migrations',

  // CLI 配置
  synchronize: false, // CLI 模式下始终禁用
  logging: true,

  // 连接配置
  timezone: '+08:00',
  charset: 'utf8mb4',
})
