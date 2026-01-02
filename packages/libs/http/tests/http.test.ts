import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest'
import { HttpClient } from '../index'

describe('HttpClient', () => {
  const base = process.env.VITE_API_BASE_PATH || 'https://api.example.com'
  let client: HttpClient

  beforeEach(() => {
    client = new HttpClient(base)
    vi.restoreAllMocks()
  })

  afterEach(() => {
    vi.clearAllMocks()
  })

  it('builds GET url with query params (including arrays) and no body', async () => {
    const mockFetch = vi.fn(async (url: RequestInfo | URL, init?: RequestInit) => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => ({
          url: String(url),
          method: init?.method,
          body: (init as any)?.body ?? null,
        }),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    const res = await client.get<any>('/users', {
      q: 'abc',
      n: 1,
      arr: [1, 2, null],
    })

    expect(res.method).toBe('GET')
    expect(res.body).toBeNull()
    expect(res.url).toContain(`${base}/users`)
    expect(res.url).toContain('q=abc')
    expect(res.url).toContain('n=1')
    // array params should repeat keys
    expect(res.url).toContain('arr=1')
    expect(res.url).toContain('arr=2')
  })

  it('merges headers and allows per-request override', async () => {
    const mockFetch = vi.fn(async (_url: RequestInfo | URL, init?: RequestInit) => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => ({ headers: init?.headers }),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    client.setHeaders({ Authorization: 'Bearer A', 'X-App': 'seed' })
    const r = await client.get<any>('/me', undefined, {
      Authorization: 'Bearer B',
    })

    const headers = r.headers as Record<string, string>
    expect(headers['Authorization']).toBe('Bearer B')
    expect(headers['X-App']).toBe('seed')
    expect(headers['Content-Type']).toBe('application/json')
  })

  it('POST json sends application/json body', async () => {
    const mockFetch = vi.fn(async (_url: RequestInfo | URL, init?: RequestInit) => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => ({
          headers: init?.headers,
          method: init?.method,
          body: (init as any)?.body,
        }),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    const payload = { name: 'Alice' }
    const r = await client.post<any>('/users', payload)

    expect(r.method).toBe('POST')
    const headers = r.headers as Record<string, string>
    expect(headers['Content-Type']).toBe('application/json')
    expect(r.body).toBe(JSON.stringify(payload))
  })

  it('upload with FormData removes json Content-Type and sends body as FormData', async () => {
    // Provide a minimal FormData in case environment lacks it
    class MockFormData {
      private _d: Record<string, any[]> = {}
      append(k: string, v: any) {
        if (!this._d[k]) this._d[k] = []
        this._d[k].push(v)
      }
      get [Symbol.toStringTag]() {
        return 'FormData'
      }
    }
    // @ts-ignore
    globalThis.FormData = (globalThis as any).FormData || (MockFormData as any)

    const mockFetch = vi.fn(async (_url: RequestInfo | URL, init?: RequestInit) => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => ({
          headers: init?.headers,
          method: init?.method,
          hasBody: !!(init as any)?.body,
        }),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    const form = new (globalThis as any).FormData()
    form.append('file', 'dummy')

    const r = await client.upload<any>('/upload', form as any)

    expect(r.method).toBe('POST')
    const headers = r.headers as Record<string, string>
    // should not force application/json when sending FormData
    expect(headers['Content-Type']).toBeUndefined()
    expect(r.hasBody).toBe(true)
  })

  it('throws parsed JSON on error response', async () => {
    const err = {
      code: 400,
      message: 'bad',
    }
    const mockFetch = vi.fn(async () => {
      return {
        ok: false,
        status: 400,
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => err,
        text: async () => 'bad',
      } as any
    })
    globalThis.fetch = mockFetch

    await expect(client.get('/err')).rejects.toMatchObject({
      status: 400,
      data: err,
    })
  })

  it('throws text on error response with text/plain', async () => {
    const mockFetch = vi.fn(async () => {
      return {
        ok: false,
        status: 500,
        headers: new Headers({ 'content-type': 'text/plain; charset=utf-8' }),
        json: async () => ({ message: 'should not be used' }),
        text: async () => 'server error',
      } as any
    })
    globalThis.fetch = mockFetch

    await expect(client.get('/err-text')).rejects.toMatchObject({
      status: 500,
      data: 'server error',
    })
  })

  it('returns undefined for 204 No Content', async () => {
    const mockFetch = vi.fn(async () => {
      return {
        ok: true,
        status: 204,
        headers: new Headers({}),
        json: async () => ({}),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    const r = await client.post<void>('/no-content')
    expect(r).toBeUndefined()
  })

  it('returns undefined for 304 Not Modified', async () => {
    const mockFetch = vi.fn(async () => {
      return {
        ok: false, // 304 is not ok per fetch
        status: 304,
        headers: new Headers({}),
        json: async () => ({}),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    const r = await client.get<void>('/not-modified')
    expect(r).toBeUndefined()
  })

  it('parses json when content-type has charset suffix', async () => {
    const mockFetch = vi.fn(async () => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({
          'content-type': 'application/json; charset=utf-8',
        }),
        json: async () => ({ ok: 1 }),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    const r = await client.get<{ ok: number }>('/ok')
    expect(r.ok).toBe(1)
  })

  it('returns text for non-json content-type', async () => {
    const mockFetch = vi.fn(async () => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({ 'content-type': 'text/plain' }),
        json: async () => 'not used',
        text: async () => 'hello',
      } as any
    })
    globalThis.fetch = mockFetch

    const r = await client.get<string>('/text')
    expect(r).toBe('hello')
  })

  it('bubbles error when response claims json but is malformed', async () => {
    const mockFetch = vi.fn(async () => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => {
          throw new Error('Invalid JSON')
        },
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    await expect(client.get('/bad-json')).rejects.toThrow('Invalid JSON')
  })

  it('DELETE can send JSON body', async () => {
    const mockFetch = vi.fn(async (_url: RequestInfo | URL, init?: RequestInit) => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => ({
          method: init?.method,
          body: (init as any)?.body,
        }),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    const r = await client.delete<any>('/items/1', { force: true })
    expect(r.method).toBe('DELETE')
    expect(r.body).toBe(JSON.stringify({ force: true }))
  })

  it('always sets CORS mode on fetch options', async () => {
    const mockFetch = vi.fn(async (_url: RequestInfo | URL, init?: RequestInit) => {
      return {
        ok: true,
        status: 200,
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => ({ mode: init?.mode }),
        text: async () => '',
      } as any
    })
    globalThis.fetch = mockFetch

    const r = await client.get<any>('/cors')
    expect(r.mode).toBe('cors')
  })
})
