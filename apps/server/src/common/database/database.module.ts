import { Module, Global, OnModuleDestroy, Inject } from '@nestjs/common'
import { ConfigModule, ConfigService } from '@nestjs/config'
import { createDrizzleConnection, type DrizzleDB } from './drizzle'
import { DRIZZLE_DB } from './drizzle.constants'
import type { Pool } from 'pg'

/**
 * 数据库模块
 * 使用 Drizzle ORM 连接 PostgreSQL
 */
@Global()
@Module({
  imports: [ConfigModule],
  providers: [
    {
      provide: DRIZZLE_DB,
      inject: [ConfigService],
      useFactory: async (configService: ConfigService) => {
        const dbConfig = configService.get('database')
        const { db, pool } = await createDrizzleConnection({
          host: dbConfig.host,
          port: dbConfig.port,
          user: dbConfig.username,
          password: dbConfig.password,
          database: dbConfig.database,
          logging: dbConfig.logging,
        })

        // 将 pool 附加到 db 实例上，用于模块销毁时关闭连接
        ;(db as any).__pool = pool

        return db
      },
    },
  ],
  exports: [DRIZZLE_DB],
})
export class DatabaseModule implements OnModuleDestroy {
  constructor(@Inject(DRIZZLE_DB) private readonly db: DrizzleDB) {}

  /**
   * 模块销毁时关闭数据库连接池
   */
  async onModuleDestroy() {
    if (this.db && (this.db as any).__pool) {
      const pool = (this.db as any).__pool as Pool
      await pool.end()
    }
  }
}
