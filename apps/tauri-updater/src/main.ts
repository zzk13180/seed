import 'reflect-metadata'
import { Logger, ValidationPipe } from '@nestjs/common'
import { NestFactory } from '@nestjs/core'
import { FastifyAdapter } from '@nestjs/platform-fastify'
import { AppModule } from './app.module'

async function bootstrap() {
  try {
    const app = await NestFactory.create(AppModule, new FastifyAdapter())

    app.enableCors()

    app.useGlobalPipes(
      new ValidationPipe({
        whitelist: true,
        transform: true,
        forbidNonWhitelisted: true,
      }),
    )

    await app.listen(3333, '0.0.0.0')
    const url = await app.getUrl()
    Logger.log(`🚀 Updater 服务启动成功: ${url}`, 'Bootstrap')
    Logger.log(`📋 配置信息:`, 'Bootstrap')
    Logger.log(`   - 端口: ${3333}`, 'Bootstrap')
    Logger.log(`   - 主机: ${'0.0.0.0'}`, 'Bootstrap')
  } catch (error) {
    Logger.error('❌ Updater 服务启动失败', (error as Error).stack, 'Bootstrap')
    throw error
  }
}

bootstrap().catch(() => void 0)
