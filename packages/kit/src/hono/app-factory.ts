/**
 * Hono 应用工厂
 *
 * 统一中间件链配置，消除 apps/api/edge 和 apps/api/bun 的重复代码。
 * 各 app 只需传入差异参数（auth 实例、环境模式、CORS 白名单）。
 *
 * @example 默认中间件全开
 * ```ts
 * const app = createHonoApp({ auth, isDev: true })
 * ```
 *
 * @example 跳过限流和日志（内部端点）
 * ```ts
 * const app = createHonoApp({ auth, isDev: true, skip: ['rateLimiter', 'logger'] })
 * ```
 */
import { Hono } from 'hono'
import { cors } from 'hono/cors'
import { secureHeaders } from 'hono/secure-headers'
import { rateLimiter } from 'hono-rate-limiter'

import {
  traceIdMiddleware,
  loggerMiddleware,
  transformMiddleware,
  errorHandlerMiddleware,
  createSessionMiddleware,
} from './middleware/index'
import type { Context } from 'hono'

/** 可跳过的中间件名称 */
export type SkippableMiddleware =
  | 'traceId'
  | 'secureHeaders'
  | 'cors'
  | 'rateLimiter'
  | 'logger'
  | 'transform'
  | 'session'

export interface HonoAppOptions {
  /** Better Auth 实例 */
  auth: {
    api: { getSession: (opts: { headers: Headers }) => Promise<any> }
    $Infer: { Session: any }
  }
  /** 是否为开发环境 */
  isDev: boolean
  /** 生产环境 CORS 允许的来源列表（开发环境自动追加 localhost 端口） */
  corsOrigins?: string[]
  /** 每分钟请求限制（默认 dev:1000 / prod:200） */
  rateLimitPerMin?: number
  /** 跳过的中间件列表（按名称） */
  skip?: SkippableMiddleware[]
}

type Variables = {
  traceId: string
  session: any
}

const DEV_ORIGINS = ['http://localhost:5173', 'http://localhost:5174', 'tauri://localhost']

/**
 * 创建配置好中间件链的 Hono 应用实例
 *
 * 中间件链: TraceId → SecureHeaders → CORS → RateLimiter → Logger → Transform → Session
 *
 * 通过 `skip` 选项按需跳过指定中间件。
 */
export function createHonoApp(options: HonoAppOptions) {
  const { auth, isDev, corsOrigins = [], rateLimitPerMin, skip = [] } = options
  const skipped = new Set(skip)
  const app = new Hono<{ Variables: Variables }>()

  // ── TraceId ──
  if (!skipped.has('traceId')) {
    app.use('*', traceIdMiddleware)
  }

  // ── 安全头 ──
  if (!skipped.has('secureHeaders')) {
    app.use(
      '*',
      secureHeaders(
        isDev
          ? {}
          : {
              contentSecurityPolicy: {
                defaultSrc: ["'self'"],
                styleSrc: ["'self'", "'unsafe-inline'"],
                imgSrc: ["'self'", 'data:', 'https:'],
                scriptSrc: ["'self'"],
              },
            },
      ),
    )
  }

  // ── CORS（生产环境必须使用白名单，不再反射任意来源）──
  if (!skipped.has('cors')) {
    const origins = isDev ? [...DEV_ORIGINS, ...corsOrigins] : corsOrigins

    if (!isDev && origins.length === 0) {
      console.warn(
        '⚠️  [createHonoApp] 生产环境未配置 corsOrigins，所有跨域请求将被拒绝。' +
          '请通过 corsOrigins 选项传入允许的来源列表。',
      )
    }

    app.use(
      '*',
      cors({
        origin: origins,
        credentials: true,
        allowHeaders: ['Content-Type', 'Authorization'],
        allowMethods: ['GET', 'POST', 'PUT', 'PATCH', 'DELETE', 'OPTIONS'],
        exposeHeaders: ['x-trace-id'],
      }),
    )
  }

  // ── 限流 ──
  if (!skipped.has('rateLimiter')) {
    app.use(
      '*',
      rateLimiter({
        windowMs: 60 * 1000,
        limit: rateLimitPerMin ?? (isDev ? 1000 : 200),
        keyGenerator: (c: Context) =>
          c.req.header('cf-connecting-ip') ??
          c.req.header('x-forwarded-for')?.split(',')[0]?.trim() ??
          'unknown',
      }),
    )
  }

  // ── 日志 ──
  if (!skipped.has('logger')) {
    app.use('*', loggerMiddleware)
  }

  // ── 响应包装 ──
  if (!skipped.has('transform')) {
    app.use('*', transformMiddleware)
  }

  // ── 会话 ──
  if (!skipped.has('session')) {
    app.use('*', createSessionMiddleware(auth))
  }

  // ── 全局错误处理（始终生效，不可跳过）──
  app.onError(errorHandlerMiddleware)

  app.notFound(c => {
    return c.json(
      {
        code: 404,
        data: null,
        message: 'Not Found',
        timestamp: Date.now(),
        path: c.req.path,
        traceId: c.get('traceId') ?? null,
      },
      404,
    )
  })

  return app
}
