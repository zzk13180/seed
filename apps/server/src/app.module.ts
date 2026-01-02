import { Module } from '@nestjs/common'
import { ConfigModule, ConfigService } from '@nestjs/config'
import { EventEmitterModule } from '@nestjs/event-emitter'
import { APP_GUARD } from '@nestjs/core'
import { ThrottlerModule, ThrottlerGuard } from '@nestjs/throttler'
import configuration from './config/configuration'
import { validationSchema } from './config/validation.schema'
import { DatabaseModule } from './common/database/database.module'
import { RedisModule } from './common/redis/redis.module'
import { JwtAuthGuard } from './common/guards/jwt-auth.guard'
import { RolesGuard } from './common/guards/roles.guard'
import { PermissionsGuard } from './common/guards/permissions.guard'
import { HealthModule } from './modules/health/health.module'
import { AuthModule } from './modules/auth/auth.module'
import { UserModule } from './modules/user/user.module'
import { TauriUpdaterModule } from './modules/tauri-updater'

/**
 * 应用程序根模块
 *
 * 这是 NestJS 应用的入口模块，负责：
 * - 注册全局配置模块
 * - 注册数据库和缓存模块
 * - 注册全局守卫（包含速率限制）
 * - 注册业务模块
 */
@Module({
  imports: [
    // 配置模块
    ConfigModule.forRoot({
      isGlobal: true, // 全局可用，无需在其他模块导入
      envFilePath: `.env.${process.env.NODE_ENV || 'development'}`,
      load: [configuration], // 加载自定义配置
      validationSchema, // 环境变量验证
    }),
    // 速率限制模块
    ThrottlerModule.forRootAsync({
      imports: [ConfigModule],
      inject: [ConfigService],
      useFactory: (configService: ConfigService) => ({
        throttlers: [
          {
            name: 'short',
            ttl: 1000, // 1 秒
            limit: configService.get<boolean>('isDev') ? 100 : 10, // 开发环境放宽限制
          },
          {
            name: 'medium',
            ttl: 10_000, // 10 秒
            limit: configService.get<boolean>('isDev') ? 200 : 50,
          },
          {
            name: 'long',
            ttl: 60_000, // 1 分钟
            limit: configService.get<boolean>('isDev') ? 1000 : 200,
          },
        ],
      }),
    }),
    EventEmitterModule.forRoot(), // 注册事件模块
    DatabaseModule, // 数据库模块
    RedisModule, // Redis 缓存模块
    HealthModule, // 健康检查模块
    AuthModule, // 认证模块
    UserModule, // 用户模块
    TauriUpdaterModule, // Tauri 应用更新模块
  ],
  controllers: [],
  providers: [
    // 全局守卫（注意顺序：Throttler -> JWT -> Roles -> Permissions）
    {
      provide: APP_GUARD,
      useClass: ThrottlerGuard,
    },
    {
      provide: APP_GUARD,
      useClass: JwtAuthGuard,
    },
    {
      provide: APP_GUARD,
      useClass: RolesGuard,
    },
    {
      provide: APP_GUARD,
      useClass: PermissionsGuard,
    },
  ],
})
export class AppModule {}
