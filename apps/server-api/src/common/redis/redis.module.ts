import { Module, Global, Logger } from '@nestjs/common'
import { CacheModule } from '@nestjs/cache-manager'
import { ConfigModule, ConfigService } from '@nestjs/config'
import { redisStore } from 'cache-manager-ioredis-yet'

/**
 * Redis 缓存模块
 *
 * - 开发环境：使用内存缓存（方便本地开发，无需 Redis）
 * - 生产环境：使用 Redis 分布式缓存
 */
@Global()
@Module({
  imports: [
    CacheModule.registerAsync({
      imports: [ConfigModule],
      inject: [ConfigService],
      useFactory: async (configService: ConfigService) => {
        const isDev = configService.get<boolean>('isDev')
        const logger = new Logger('RedisModule')

        // 开发环境使用内存缓存
        if (isDev) {
          logger.log('Using in-memory cache (development mode)')
          return {
            ttl: 60 * 60 * 1000, // 1 小时 (毫秒)
            max: 100, // 最大缓存项
          }
        }

        // 生产环境使用 Redis
        const redisConfig = configService.get('redis')
        logger.log(`Connecting to Redis at ${redisConfig.host}:${redisConfig.port}`)

        try {
          const store = await redisStore({
            host: redisConfig.host,
            port: redisConfig.port,
            password: redisConfig.password || undefined,
            db: redisConfig.db,
            ttl: 60 * 60, // 1 小时（秒）
          })

          logger.log('Redis connection established')
          return { store }
        } catch (error) {
          logger.error('Failed to connect to Redis, falling back to memory cache', error)
          // Redis 连接失败时降级为内存缓存
          return {
            ttl: 60 * 60 * 1000,
            max: 1000,
          }
        }
      },
    }),
  ],
  exports: [CacheModule],
})
export class RedisModule {}
