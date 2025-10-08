import { Module, Global } from '@nestjs/common'
import { TypeOrmModule } from '@nestjs/typeorm'
import { ConfigModule, ConfigService } from '@nestjs/config'
import { AuditSubscriber } from '../subscribers/audit.subscriber'

/**
 * 数据库模块
 */
@Global()
@Module({
  imports: [
    TypeOrmModule.forRootAsync({
      imports: [ConfigModule],
      inject: [ConfigService],
      useFactory: (configService: ConfigService) => {
        const dbConfig = configService.get('database')
        return {
          type: dbConfig.type,
          host: dbConfig.host,
          port: dbConfig.port,
          username: dbConfig.username,
          password: dbConfig.password,
          database: dbConfig.database,
          entities: [`${__dirname}/../../**/*.entity{.ts,.js}`],
          // 注册 TypeORM Subscriber
          subscribers: [AuditSubscriber],
          synchronize: dbConfig.synchronize,
          logging: dbConfig.logging,
          timezone: '+08:00', // 东八区时间
          charset: 'utf8mb4',
          extra: {
            connectionLimit: 10,
          },
        }
      },
    }),
  ],
})
export class DatabaseModule {}
