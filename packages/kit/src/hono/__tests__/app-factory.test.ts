/**
 * createHonoApp 工厂测试
 *
 * 覆盖:
 * - 默认中间件链完整注册
 * - skip 选项跳过指定中间件
 * - CORS 生产环境空白名单警告
 * - 404 处理
 */
import { describe, it, expect, vi, beforeEach } from 'vitest'
import { createHonoApp, type HonoAppOptions, type SkippableMiddleware } from '../app-factory'

// ── Mock 外部依赖 ──

// 跟踪哪些中间件被注册
const registeredMiddleware: string[] = []

// Mock rate-limiter（避免真实初始化）
vi.mock('hono-rate-limiter', () => ({
  rateLimiter: vi.fn(() => {
    return async (_c: any, next: any) => {
      registeredMiddleware.push('rateLimiter')
      await next()
    }
  }),
}))

// Mock 内部中间件
vi.mock('../middleware/index', () => ({
  traceIdMiddleware: async (c: any, next: any) => {
    registeredMiddleware.push('traceId')
    c.set('traceId', 'test-trace-id')
    await next()
  },
  loggerMiddleware: async (_c: any, next: any) => {
    registeredMiddleware.push('logger')
    await next()
  },
  transformMiddleware: async (_c: any, next: any) => {
    registeredMiddleware.push('transform')
    await next()
  },
  errorHandlerMiddleware: vi.fn((err: Error, c: any) => {
    return c.json({ code: 500, message: err.message }, 500)
  }),
  createSessionMiddleware: vi.fn(() => {
    return async (c: any, next: any) => {
      registeredMiddleware.push('session')
      c.set('session', null)
      await next()
    }
  }),
}))

const mockAuth: HonoAppOptions['auth'] = {
  api: { getSession: vi.fn(async () => null) },
  $Infer: { Session: {} as any },
}

function createDefaultOptions(overrides: Partial<HonoAppOptions> = {}): HonoAppOptions {
  return { auth: mockAuth, isDev: true, ...overrides }
}

describe('createHonoApp', () => {
  beforeEach(() => {
    registeredMiddleware.length = 0
  })

  describe('default middleware chain', () => {
    it('should register all middleware in order', async () => {
      const app = createHonoApp(createDefaultOptions())

      // 添加一个测试路由
      app.get('/test', c => c.json({ ok: true }))

      const res = await app.request('/test')
      expect(res.status).toBe(200)

      // 验证中间件按顺序执行
      expect(registeredMiddleware).toContain('traceId')
      expect(registeredMiddleware).toContain('rateLimiter')
      expect(registeredMiddleware).toContain('logger')
      expect(registeredMiddleware).toContain('transform')
      expect(registeredMiddleware).toContain('session')
    })
  })

  describe('skip option', () => {
    it('should skip specified middleware', async () => {
      const skip: SkippableMiddleware[] = ['rateLimiter', 'logger', 'session']
      const app = createHonoApp(createDefaultOptions({ skip }))

      app.get('/test', c => c.json({ ok: true }))

      await app.request('/test')

      expect(registeredMiddleware).toContain('traceId')
      expect(registeredMiddleware).toContain('transform')
      expect(registeredMiddleware).not.toContain('rateLimiter')
      expect(registeredMiddleware).not.toContain('logger')
      expect(registeredMiddleware).not.toContain('session')
    })

    it('should skip all skippable middleware', async () => {
      const skip: SkippableMiddleware[] = [
        'traceId',
        'secureHeaders',
        'cors',
        'rateLimiter',
        'logger',
        'transform',
        'session',
      ]
      const app = createHonoApp(createDefaultOptions({ skip }))

      app.get('/test', c => c.json({ ok: true }))

      await app.request('/test')

      expect(registeredMiddleware).toHaveLength(0)
    })
  })

  describe('CORS production warning', () => {
    it('should warn when production has no corsOrigins', () => {
      const warnSpy = vi.spyOn(console, 'warn').mockImplementation(() => {})

      createHonoApp(createDefaultOptions({ isDev: false, corsOrigins: [] }))

      expect(warnSpy).toHaveBeenCalledWith(expect.stringContaining('生产环境未配置 corsOrigins'))
      warnSpy.mockRestore()
    })

    it('should not warn in dev mode', () => {
      const warnSpy = vi.spyOn(console, 'warn').mockImplementation(() => {})

      createHonoApp(createDefaultOptions({ isDev: true, corsOrigins: [] }))

      expect(warnSpy).not.toHaveBeenCalled()
      warnSpy.mockRestore()
    })

    it('should not warn when corsOrigins are provided', () => {
      const warnSpy = vi.spyOn(console, 'warn').mockImplementation(() => {})

      createHonoApp(
        createDefaultOptions({
          isDev: false,
          corsOrigins: ['https://example.com'],
        }),
      )

      expect(warnSpy).not.toHaveBeenCalled()
      warnSpy.mockRestore()
    })
  })

  describe('404 handler', () => {
    it('should return standardized 404 JSON', async () => {
      const app = createHonoApp(createDefaultOptions())

      const res = await app.request('/nonexistent')
      expect(res.status).toBe(404)

      const body = await res.json()
      expect(body.code).toBe(404)
      expect(body.message).toBe('Not Found')
      expect(body.path).toBe('/nonexistent')
      expect(body).toHaveProperty('traceId')
      expect(body).toHaveProperty('timestamp')
    })
  })

  describe('error handler', () => {
    it('should always register error handler even with full skip', async () => {
      const skip: SkippableMiddleware[] = [
        'traceId',
        'secureHeaders',
        'cors',
        'rateLimiter',
        'logger',
        'transform',
        'session',
      ]
      const app = createHonoApp(createDefaultOptions({ skip }))

      app.get('/error', () => {
        throw new Error('test error')
      })

      const res = await app.request('/error')
      expect(res.status).toBe(500)
    })
  })
})
