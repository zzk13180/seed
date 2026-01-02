import { Injectable, NestMiddleware } from '@nestjs/common'

/**
 * 生成简单的唯一 ID
 */
function generateTraceId(): string {
  const timestamp = Date.now().toString(36)
  const randomPart = Math.random().toString(36).slice(2, 10)
  return `${timestamp}-${randomPart}`
}

/**
 * 请求追踪 ID 中间件
 *
 * 为每个请求生成唯一的追踪 ID，便于日志关联和问题排查
 */
@Injectable()
export class TraceIdMiddleware implements NestMiddleware {
  use(req: any, res: any, next: () => void): void {
    // 从请求头获取或生成新的追踪 ID
    const traceId =
      (req.headers['x-trace-id'] as string) ||
      (req.headers['x-request-id'] as string) ||
      generateTraceId()

    // 将追踪 ID 添加到请求对象
    req.traceId = traceId

    // 将追踪 ID 添加到响应头
    res.setHeader('x-trace-id', traceId)

    next()
  }
}

/**
 * Fastify 钩子版本的追踪 ID 处理
 * 用于 Fastify 原生钩子
 */
export function traceIdHook(request: any, reply: any, done: () => void): void {
  const traceId =
    (request.headers['x-trace-id'] as string) ||
    (request.headers['x-request-id'] as string) ||
    generateTraceId()

  // 将追踪 ID 添加到请求对象
  request.traceId = traceId
  if (request.raw) {
    request.raw.traceId = traceId
  }

  // 将追踪 ID 添加到响应头
  reply.header('x-trace-id', traceId)

  done()
}
