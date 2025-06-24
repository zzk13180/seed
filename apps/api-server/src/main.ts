import { join } from 'node:path'
import { NestFactory } from '@nestjs/core'
import { FastifyAdapter } from '@nestjs/platform-fastify'
import { WsAdapter } from '@nestjs/platform-ws'
import { AppModule } from './app.module'
import type { NestFastifyApplication } from '@nestjs/platform-fastify'

/**
 * 应用程序启动函数
 *
 * 配置和启动 NestJS 应用服务器，支持：
 * - HTTP 服务（使用 Fastify 适配器提高性能）
 * - WebSocket 服务（使用 ws 适配器）
 * - 静态文件服务
 * - 优雅关停
 */
async function bootstrap() {
  try {
    // 使用 FastifyAdapter 创建应用实例
    // Fastify 相比 Express 有更好的性能表现
    const app = await NestFactory.create<NestFastifyApplication>(AppModule, new FastifyAdapter())

    // 配置静态资源服务
    // 将 public 目录下的文件作为静态资源提供服务
    app.useStaticAssets({
      root: join(__dirname, './', 'public'), // 静态文件根目录
      prefix: '/', // URL 前缀
      maxAge: '1h', // 缓存时间
    })

    // 使用 ws 适配器来支持 WebSocket
    // 这使得应用可以同时处理 HTTP 和 WebSocket 连接
    app.useWebSocketAdapter(new WsAdapter(app))

    // 启用优雅关停
    // 确保应用在接收到终止信号时能正确清理资源
    app.enableShutdownHooks()

    // 启动服务器
    // 监听所有网络接口的 3003 端口
    await app.listen(3003, '0.0.0.0')

    console.log(`🚀 服务已启动: ${await app.getUrl()}`)
    console.log(`📁 静态文件服务: ${await app.getUrl()}/`)
    console.log(`🔌 WebSocket 地址: ws://localhost:3003/ws/robot`)
  } catch (error) {
    console.error('❌ 服务启动失败:', error)
    process.exit(1)
  }
}

// 启动应用程序
// eslint-disable-next-line @typescript-eslint/no-floating-promises
bootstrap()
