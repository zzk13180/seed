import { Hono } from 'hono'
import { serve } from '@hono/node-server'
import { z } from 'zod'
import { zValidator } from '@hono/zod-validator'
import { logger } from 'hono/logger'

const app = new Hono()

app.get('/', c => {
  return c.text('Hello')
})

const schema = z.object({
  'Target-URL': z.string().url(),
})

app.all('/proxy/*', logger(), zValidator('header', schema), async c => {
  const originalUrl = new URL(c.req.url)
  const targetUrl = c.req.header('Target-URL')
  const method = c.req.method
  const originalPath = c.req.path
  const queryString = originalUrl.search
  const proxiedUrl = `${targetUrl}${originalPath}${queryString}`
  let requestBody: any

  // 1. 处理 DELETE/OPTIONS 请求的 content-length
  const clientContentType = c.req.header('Content-Type')
  const clientContentLength = c.req.header('Content-Length')
  if ((method === 'DELETE' || method === 'OPTIONS') && !clientContentLength) {
    // 某些服务端需要 content-length: 0
    c.req.raw.headers['content-length'] = '0'
    delete c.req.raw.headers['transfer-encoding']
  }

  // 2. 读取请求体
  if (method !== 'GET' && method !== 'HEAD') {
    try {
      if (
        (clientContentLength && clientContentLength !== '0') ||
        (clientContentType &&
          (clientContentType.includes('json') ||
            clientContentType.includes('x-www-form-urlencoded') ||
            clientContentType.includes('multipart/form-data')))
      ) {
        requestBody = await c.req.arrayBuffer()
      }
    } catch (error: any) {
      console.warn(
        `[Proxy Warning] Could not read request body for ${method} ${originalPath}: ${error.message}`,
      )
    }
  }

  // 3. 构建转发请求头（最小可用头部转发）
  const headersToForward = new Headers()
  const contentTypeFromClient = c.req.header('Content-Type')
  if (contentTypeFromClient) {
    headersToForward.set('Content-Type', contentTypeFromClient)
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
    const timeout = setTimeout(() => controller.abort(), 60_000) // 60s 超时

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
        // Hono/Fetch Headers 只支持 append
        responseHeaders.append('set-cookie', value)
      } else {
        responseHeaders.set(key, value)
      }
    }

    // 处理重定向 Location
    if (responseHeaders.has('location')) {
      try {
        const loc = new URL(responseHeaders.get('location'), targetUrl)
        // 如果目标 host 与 location host 相同，重写为当前服务 host
        if (loc.host === new URL(targetUrl).host) {
          loc.host = c.req.header('host') || loc.host
          loc.protocol = originalUrl.protocol
          responseHeaders.set('location', loc.toString())
        }
      } catch {}
    }

    // 处理 HTTP/1.0 Transfer-Encoding
    if (c.req.raw?.raw?.httpVersion === '1.0') {
      responseHeaders.delete('transfer-encoding')
      responseHeaders.set('connection', c.req.header('connection') || 'close')
    }

    return new Response(targetResponse.body, {
      status: targetResponse.status,
      statusText: targetResponse.statusText,
      headers: responseHeaders,
    })
  } catch (error: any) {
    console.error(`[Proxy Error] Failed to fetch ${proxiedUrl}:`, error.message)
    if (error.message.includes('ECONNREFUSED')) {
      return c.json({ error: 'Proxy request failed: Connection refused by target server' }, 502)
    }
    return c.json({ error: 'Proxy request failed', details: error.message }, 502)
  }
})

serve(
  {
    fetch: app.fetch,
    port: 3000,
  },
  info => {
    console.log(`Server is running on http://localhost:${info.port}`)
  },
)
