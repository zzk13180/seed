import { Hono } from 'hono'
import { serve } from '@hono/node-server'
import { z } from 'zod'
import { zValidator } from '@hono/zod-validator'
import { logger } from 'hono/logger'
import type { Context } from 'hono'

/**
 * 代理服务配置
 */
const CONFIG = {
  PORT: Number(process.env.PORT) || 3000,
  REQUEST_TIMEOUT: 60_000, // 60 秒
} as const

/**
 * 请求头验证 Schema
 */
const headerSchema = z.object({
  'Target-URL': z.string().url(),
})

/**
 * 无需请求体的 HTTP 方法
 */
const METHODS_WITHOUT_BODY = new Set(['GET', 'HEAD'])

/**
 * 需要处理 content-length 的方法
 */
const METHODS_NEEDING_CONTENT_LENGTH = new Set(['DELETE', 'OPTIONS'])

const app = new Hono()

/**
 * 健康检查端点
 */
app.get('/', (c: Context) => {
  return c.json({
    status: 'ok',
    service: 'seed-proxy',
    timestamp: new Date().toISOString(),
  })
})

/**
 * 健康检查端点
 */
app.get('/health', (c: Context) => {
  return c.json({ status: 'ok' })
})

/**
 * 代理请求处理
 */
app.all('/proxy/*', logger(), zValidator('header', headerSchema), async (c: Context) => {
  const originalUrl = new URL(c.req.url)
  const targetUrl = c.req.header('Target-URL')
  const method = c.req.method
  const originalPath = c.req.path
  const queryString = originalUrl.search
  const proxiedUrl = `${targetUrl}${originalPath}${queryString}`

  let requestBody: ArrayBuffer | undefined

  // 1. 处理 DELETE/OPTIONS 请求的 content-length
  const clientContentType = c.req.header('Content-Type')
  const clientContentLength = c.req.header('Content-Length')
  if (METHODS_NEEDING_CONTENT_LENGTH.has(method) && !clientContentLength) {
    const rawHeaders = c.req.raw.headers as any
    rawHeaders['content-length'] = '0'
    delete rawHeaders['transfer-encoding']
  }

  // 2. 读取请求体
  if (!METHODS_WITHOUT_BODY.has(method)) {
    try {
      const hasContent =
        (clientContentLength && clientContentLength !== '0') ||
        (clientContentType &&
          (clientContentType.includes('json') ||
            clientContentType.includes('x-www-form-urlencoded') ||
            clientContentType.includes('multipart/form-data')))

      if (hasContent) {
        requestBody = await c.req.arrayBuffer()
      }
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error'
      console.warn(
        `[Proxy Warning] Could not read request body for ${method} ${originalPath}: ${errorMessage}`,
      )
    }
  }

  // 3. 构建转发请求头（最小可用头部转发）
  const headersToForward = new Headers()
  if (clientContentType) {
    headersToForward.set('Content-Type', clientContentType)
  }
  const acceptHeaderFromClient = c.req.header('Accept')
  if (acceptHeaderFromClient) {
    headersToForward.set('Accept', acceptHeaderFromClient)
  } else {
    headersToForward.set('Accept', 'application/json')
  }

  // 4. 发起转发
  try {
    const controller = new AbortController()
    const timeout = setTimeout(() => controller.abort(), CONFIG.REQUEST_TIMEOUT)

    const targetResponse = await fetch(proxiedUrl, {
      method,
      headers: headersToForward,
      body: requestBody,
      redirect: 'manual',
      signal: controller.signal,
    })
    clearTimeout(timeout)

    // 5. 处理响应头
    const responseHeaders = new Headers()
    for (const [key, value] of targetResponse.headers.entries()) {
      // 处理 set-cookie 多值
      if (key.toLowerCase() === 'set-cookie') {
        responseHeaders.append('set-cookie', value)
      } else {
        responseHeaders.set(key, value)
      }
    }

    // 处理重定向 Location
    if (responseHeaders.has('location')) {
      try {
        const locationUrl = responseHeaders.get('location')
        if (!locationUrl) throw new Error('Invalid location header')
        const loc = new URL(locationUrl, targetUrl ?? undefined)
        // 如果目标 host 与 location host 相同，重写为当前服务 host
        if (loc.host === new URL(targetUrl ?? '').host) {
          loc.host = c.req.header('host') || loc.host
          loc.protocol = originalUrl.protocol
          responseHeaders.set('location', loc.toString())
        }
      } catch {
        // 忽略无效的 Location URL
      }
    }

    const rawRequest = c.req.raw as any
    if (rawRequest?.raw?.httpVersion === '1.0') {
      responseHeaders.delete('transfer-encoding')
      responseHeaders.set('connection', c.req.header('connection') || 'close')
    }

    return new Response(targetResponse.body, {
      status: targetResponse.status,
      statusText: targetResponse.statusText,
      headers: responseHeaders,
    })
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error'
    console.error(`[Proxy Error] Failed to fetch ${proxiedUrl}:`, errorMessage)

    if (errorMessage.includes('ECONNREFUSED')) {
      return c.json({ error: 'Proxy request failed: Connection refused by target server' }, 502)
    }

    return c.json({ error: 'Proxy request failed', details: errorMessage }, 502)
  }
})

/**
 * 启动服务器
 */
serve(
  {
    fetch: app.fetch,
    port: CONFIG.PORT,
  },
  info => {
    console.log(`Server is running on http://localhost:${info.port}`)
  },
)
