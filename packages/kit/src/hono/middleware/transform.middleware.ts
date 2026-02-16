import { ResponseCode } from '@seed/contracts/errors/response-code'
import type { Context, Next } from 'hono'

/**
 * 统一响应包装中间件
 *
 * 自动将成功响应包装为 { code, data, message, timestamp, traceId } 格式
 * 跳过 Better Auth 路由和非 JSON 响应
 */
export async function transformMiddleware(c: Context, next: Next) {
  await next()

  const path = c.req.path
  if (
    path.startsWith('/api/auth/') ||
    c.res.status >= 400 ||
    c.res.status === 204 ||
    c.res.headers.get('x-response-transformed') === 'true'
  ) {
    return
  }

  const contentType = c.res.headers.get('content-type') ?? ''
  if (!contentType.includes('application/json')) {
    return
  }

  try {
    const originalBody = await c.res.json()
    const traceId = c.get('traceId') ?? null

    const wrapped = {
      code: ResponseCode.SUCCESS,
      data: originalBody,
      message: 'Success',
      timestamp: Date.now(),
      traceId,
    }

    const headers = new Headers(c.res.headers)
    headers.set('content-type', 'application/json')
    headers.set('x-response-transformed', 'true')
    headers.set('x-trace-id', traceId ?? '')

    c.res = new Response(JSON.stringify(wrapped), {
      status: c.res.status,
      headers,
    })
  } catch {
    // 无法解析 JSON，保持原样
  }
}
