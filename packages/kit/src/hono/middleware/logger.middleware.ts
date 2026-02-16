import type { Context, Next } from 'hono'

/**
 * 请求日志中间件
 *
 * 记录请求方法、URL、IP、User-Agent、耗时
 * 敏感字段自动脱敏
 */
export async function loggerMiddleware(c: Context, next: Next) {
  const { method, path } = c.req
  const ip =
    c.req.header('x-forwarded-for')?.split(',')[0]?.trim() ?? c.req.header('x-real-ip') ?? 'unknown'
  const ua = (c.req.header('user-agent') ?? '').slice(0, 80)
  const traceId = c.get('traceId') ?? '-'
  const start = Date.now()

  console.log(`[${traceId}] --> ${method} ${path} | IP: ${ip} | UA: ${ua}`)

  // 非 GET 请求记录请求体（脱敏后）
  if (method !== 'GET') {
    try {
      const body = await c.req.raw.clone().json()
      if (body && typeof body === 'object') {
        const sanitized = sanitizeBody(body as Record<string, unknown>)
        console.debug(`[${traceId}] Body: ${JSON.stringify(sanitized).slice(0, 500)}`)
      }
    } catch {
      // 非 JSON body，忽略
    }
  }

  await next()

  const cost = Date.now() - start
  const status = c.res.status
  const level = status >= 400 ? 'warn' : 'log'
  console[level](`[${traceId}] <-- ${method} ${path} | ${status} | ${cost}ms`)
}

const SENSITIVE_FIELDS = new Set([
  'password',
  'newPassword',
  'oldPassword',
  'confirmPassword',
  'token',
  'refreshToken',
  'accessToken',
  'secret',
  'creditCard',
  'cvv',
])

function sanitizeBody(body: Record<string, unknown>, depth = 0): Record<string, unknown> {
  if (depth > 3) return { '...': '[truncated]' }
  const sanitized = { ...body }
  for (const field of Object.keys(sanitized)) {
    if (SENSITIVE_FIELDS.has(field)) {
      sanitized[field] = '******'
    } else if (
      sanitized[field] !== null &&
      typeof sanitized[field] === 'object' &&
      !Array.isArray(sanitized[field])
    ) {
      sanitized[field] = sanitizeBody(sanitized[field] as Record<string, unknown>, depth + 1)
    }
  }
  return sanitized
}
