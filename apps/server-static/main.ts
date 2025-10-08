/* eslint-disable unicorn/no-process-exit */
import { join, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
import os from 'node:os'
import fastify from 'fastify'
import fastifyStatic from '@fastify/static'

/**
 * 获取环境变量，支持默认值
 */
function getEnvVar(key: string, defaultValue: string): string {
  return process.env[key] || defaultValue
}

/**
 * 获取本机局域网 IP 地址
 */
function getLanIp(): string {
  const interfaces = os.networkInterfaces()
  for (const name of Object.keys(interfaces)) {
    const ifaceList = interfaces[name]
    if (!ifaceList) continue
    for (const iface of ifaceList) {
      if (iface.family === 'IPv4' && !iface.internal) {
        return iface.address
      }
    }
  }
  return 'localhost'
}

/**
 * 获取 PUBLIC 目录路径，支持多种部署方式
 */
function getPublicDir(): string {
  // 支持通过环境变量自定义静态文件目录
  const customDir = process.env.PUBLIC_DIR
  if (customDir) {
    return customDir
  }

  const baseDir = dirname(fileURLToPath(import.meta.url))
  console.log('Base Directory:', baseDir)
  return join(baseDir, 'public')
}

/**
 * 创建并启动 Fastify 服务器
 */
async function startServer(): Promise<void> {
  const port = parseInt(getEnvVar('PORT', '8080'), 10)
  const host = getEnvVar('HOST', '0.0.0.0')
  const publicDir = getPublicDir()
  const maxAge = parseInt(getEnvVar('CACHE_MAX_AGE', '3600000'), 10) // 默认 1 小时

  const app = fastify({
    logger: {
      level: process.env.LOG_LEVEL || 'info',
      transport: {
        target: 'pino-pretty',
        options: {
          colorize: true,
          translateTime: 'SYS:standard',
          ignore: 'pid,hostname',
        },
      },
    },
  })

  try {
    // 注册静态资源服务
    await app.register(fastifyStatic, {
      root: publicDir,
      prefix: '/',
      maxAge,
      constraints: {},
    })

    // 健康检查路由
    app.get('/health', async (request, reply) => {
      return reply.send({ status: 'ok', timestamp: new Date().toISOString() })
    })

    // API 版本信息
    app.get('/api/version', async (request, reply) => {
      return reply.send({ version: '0.0.1', timestamp: new Date().toISOString() })
    })

    // 启动服务器
    await app.listen({ port, host })

    // 获取局域网 IP
    const lanIp = getLanIp()

    // 输出启动信息
    const separator = '='.repeat(50)
    console.log(`\n${separator}`)
    console.log('🚀 Assets 服务已启动')
    console.log(separator)
    console.log(`📍 本地访问地址: http://localhost:${port}`)
    console.log(`🌐 局域网访问地址: http://${lanIp}:${port}`)
    console.log(`🔍 健康检查: http://${lanIp}:${port}/health`)
    console.log(`📦 静态文件目录: ${publicDir}`)
    console.log(`⚙️  环境配置: HOST=${host}, PORT=${port}, CACHE_MAX_AGE=${maxAge}ms`)
    console.log(`${separator}\n`)

    // Graceful shutdown 处理
    const gracefulShutdown = async (signal: string) => {
      console.log(`\n📡 收到 ${signal} 信号，开始优雅关闭...`)
      try {
        await app.close()
        console.log('✅ 服务已安全关闭')
        process.exit(0)
      } catch (error) {
        console.error('❌ 关闭过程中出错:', error)
        process.exit(1)
      }
    }

    process.on('SIGTERM', () => {
      gracefulShutdown('SIGTERM').catch(console.error)
    })
    process.on('SIGINT', () => {
      gracefulShutdown('SIGINT').catch(console.error)
    })
  } catch (error) {
    console.error('❌ 服务启动失败:')
    console.error(error)
    process.exit(1)
  }
}

/**
 * 注册全局异常处理器
 */
function registerExceptionHandlers(): void {
  process.on('uncaughtException', (error: Error) => {
    console.error('💥 未捕获的异常:')
    console.error(error)
    process.exit(1)
  })

  process.on('unhandledRejection', (reason: unknown, promise: Promise<unknown>) => {
    console.error('⚠️ 未处理的 Promise 拒绝:')
    console.error('Promise:', promise)
    console.error('Reason:', reason)
    process.exit(1)
  })
}

// 注册异常处理器
registerExceptionHandlers()

// 启动服务器
await startServer().catch((error: Error) => {
  console.error('❌ 服务启动失败:', error)
  process.exit(1)
})
