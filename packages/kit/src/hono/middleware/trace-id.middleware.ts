import type { Context, Next } from 'hono'

/**
 * 请求追踪 ID 中间件
 *
 * 从请求头获取或生成唯一追踪 ID，注入到上下文并写入响应头
 */
export async function traceIdMiddleware(c: Context, next: Next) {
  const id = c.req.header('x-trace-id') ?? c.req.header('x-request-id') ?? generateTraceId()

  c.set('traceId', id)
  c.header('x-trace-id', id)

  await next()
}

function generateTraceId(): string {
  const timestamp = Date.now().toString(36)
  const randomPart = Math.random().toString(36).slice(2, 10)
  return `${timestamp}-${randomPart}`
}
