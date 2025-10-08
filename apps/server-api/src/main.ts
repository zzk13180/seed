import { join } from 'node:path'
import { NestFactory, HttpAdapterHost } from '@nestjs/core'
import { FastifyAdapter } from '@nestjs/platform-fastify'
import { WsAdapter } from '@nestjs/platform-ws'
import { ConfigService } from '@nestjs/config'
import { ValidationPipe, Logger } from '@nestjs/common'
import { DocumentBuilder, SwaggerModule } from '@nestjs/swagger'
import helmet from '@fastify/helmet'
import { AppModule } from './app.module'
import { TransformInterceptor } from './common/interceptors/transform.interceptor'
import { AuditInterceptor } from './common/interceptors/audit.interceptor'
import { LoggingInterceptor } from './common/interceptors/logging.interceptor'
import { AllExceptionsFilter } from './common/filters/all-exceptions.filter'
import { traceIdHook } from './common/middleware/trace-id.middleware'
import type { NestFastifyApplication } from '@nestjs/platform-fastify'

/**
 * 应用程序启动函数
 */
async function bootstrap() {
  const logger = new Logger('Bootstrap')
  try {
    // 使用 FastifyAdapter 创建应用实例
    const app = await NestFactory.create<NestFastifyApplication>(AppModule, new FastifyAdapter())
    const configService = app.get(ConfigService)
    const port = configService.get<number>('port') || 3000
    const apiPrefix = configService.get<string>('apiPrefix') || 'api'
    const isDev = configService.get<boolean>('isDev')

    // 设置全局路由前缀
    app.setGlobalPrefix(apiPrefix)

    // 注册追踪 ID 钩子
    const fastifyInstance = app.getHttpAdapter().getInstance()
    fastifyInstance.addHook('onRequest', traceIdHook)

    // 注册 Helmet 安全中间件（设置安全 HTTP 头）
    await app.register(helmet, {
      // 生产环境启用严格 CSP，开发环境放宽以支持 Swagger UI
      contentSecurityPolicy: isDev
        ? false
        : {
            directives: {
              defaultSrc: ["'self'"],
              styleSrc: ["'self'", "'unsafe-inline'"],
              imgSrc: ["'self'", 'data:', 'https:'],
              scriptSrc: ["'self'"],
            },
          },
    })

    // 配置静态资源服务
    app.useStaticAssets({
      root: join(__dirname, './', 'public'),
      prefix: '/',
      maxAge: '1h',
    })

    // 使用 ws 适配器来支持 WebSocket
    app.useWebSocketAdapter(new WsAdapter(app))

    // 全局管道、拦截器、过滤器
    app.useGlobalPipes(
      new ValidationPipe({
        transform: true, // 自动转换参数类型
        whitelist: true, // 自动剔除非 DTO 属性
        forbidNonWhitelisted: true, // 禁止非白名单属性，抛出错误
        forbidUnknownValues: true, // 禁止未知值
        stopAtFirstError: true, // 遇到第一个错误即停止验证
      }),
    )
    // 注意：拦截器的顺序很重要
    // LoggingInterceptor -> AuditInterceptor -> TransformInterceptor
    app.useGlobalInterceptors(
      new LoggingInterceptor(),
      new AuditInterceptor(),
      new TransformInterceptor(),
    )

    const httpAdapter = app.get(HttpAdapterHost)
    app.useGlobalFilters(new AllExceptionsFilter(httpAdapter))

    // 启用 CORS
    app.enableCors({
      origin: true,
      credentials: true,
    })

    // Swagger 文档配置
    const config = new DocumentBuilder()
      .setTitle('Seed API')
      .setDescription('Seed 全栈种子项目 - NestJS 后端 API 文档')
      .setVersion('1.0')
      .addBearerAuth()
      .build()
    const document = SwaggerModule.createDocument(app, config)
    SwaggerModule.setup('docs', app, document)

    // 启用优雅关停
    app.enableShutdownHooks()

    // 启动服务器
    await app.listen(port, '0.0.0.0')

    const url = await app.getUrl()
    logger.log(`🚀 Application is running on: ${url}/${apiPrefix}`)
    logger.log(`📚 Swagger documentation: ${url}/docs`)
    logger.log(`📁 Static assets: ${url}/`)
    logger.log(`🔌 WebSocket URL: ws://localhost:${port}/ws/robot`)
  } catch (error) {
    logger.error('❌ Application failed to start', error)

    process.exit(1)
  }
}

// 启动应用程序
// eslint-disable-next-line @typescript-eslint/no-floating-promises
bootstrap()
