import { describe, it, expect, vi, afterEach } from 'vitest'
import { encodeBase64, decodeBase64 } from '../base64.util'

// Helper to compute expected base64 using Node's Buffer
const toBase64 = (s: string) => Buffer.from(s, 'utf8').toString('base64')
const fromBase64 = (b64: string) => Buffer.from(b64, 'base64').toString('utf8')

// Provide polyfills for atob / btoa when running in Node environment (without jsdom)
const g: any = globalThis as any
if (!g.atob) {
  g.atob = (b64: string) => Buffer.from(b64, 'base64').toString('binary')
}
if (!g.btoa) {
  g.btoa = (str: string) => Buffer.from(str, 'binary').toString('base64')
}

describe('base64.util', () => {
  afterEach(() => {
    vi.restoreAllMocks()
  })

  describe('encodeBase64', () => {
    it('returns empty string for empty input', () => {
      expect(encodeBase64('')).toBe('')
    })

    it('encodes ASCII text correctly', () => {
      expect(encodeBase64('hello')).toBe('aGVsbG8=')
      expect(encodeBase64('Hello World!')).toBe('SGVsbG8gV29ybGQh')
    })

    it('encodes unicode text correctly (CJK, emoji, mixed)', () => {
      const cases = [
        '你好，世界',
        'こんにちは世界',
        '안녕하세요 세계',
        '😀😃😄😁😆',
        '👨‍👩‍👧‍👦 family',
        '汉字 and Latin 123',
        '\u0000\u0001\u0002 nulls',
      ]
      for (const s of cases) {
        expect(encodeBase64(s)).toBe(toBase64(s))
      }
    })

    it('handles very long strings (>32KB) due to chunking', () => {
      const long = '汉' // 3 bytes in UTF-8
      const input = long.repeat(40000) // ~120KB
      const actual = encodeBase64(input)
      const expected = toBase64(input)
      expect(actual).toBe(expected)
      // sanity: round-trip
      expect(decodeBase64(actual)).toBe(input)
    })

    it('propagates error as friendly message when TextEncoder fails', () => {
      const OriginalTextEncoder = globalThis.TextEncoder
      // Mock TextEncoder.encode to throw
      // @ts-expect-error override for test
      globalThis.TextEncoder = class {
        encode() {
          throw new Error('boom')
        }
      }
      try {
        expect(() => encodeBase64('x')).toThrowError('Base64 编码失败')
      } finally {
        if (OriginalTextEncoder) {
          globalThis.TextEncoder = OriginalTextEncoder
        }
      }
    })
  })

  describe('decodeBase64', () => {
    it('returns empty string for empty input', () => {
      expect(decodeBase64('')).toBe('')
    })

    it('decodes valid base64 strings', () => {
      const pairs: Array<[string, string]> = [
        ['aGVsbG8=', 'hello'],
        ['SGVsbG8gV29ybGQh', 'Hello World!'],
        [toBase64('你好'), '你好'],
        [toBase64('😀😃😄'), '😀😃😄'],
      ]
      for (const [b64, text] of pairs) {
        expect(decodeBase64(b64)).toBe(text)
      }
    })

    it('rejects invalid base64 format (length not multiple of 4)', () => {
      expect(() => decodeBase64('aGVsbG8')).toThrowError('无效的 Base64 格式')
    })

    it('rejects invalid characters', () => {
      expect(() => decodeBase64('abc$')).toThrowError('无效的 Base64 格式')
      expect(() => decodeBase64('###=')).toThrowError('无效的 Base64 格式')
    })

    it("rejects '=' in the middle and excessive padding", () => {
      expect(() => decodeBase64('ab=cd')).toThrowError('无效的 Base64 格式')
      expect(() => decodeBase64('A===')).toThrowError('无效的 Base64 格式')
      expect(() => decodeBase64('====')).toThrowError('无效的 Base64 格式')
    })

    it('propagates decode failures with friendly message when atob throws', () => {
      const spy = vi.spyOn(globalThis as any, 'atob').mockImplementation(() => {
        throw new Error('native fail')
      })
      expect(() => decodeBase64('aGVsbG8=')).toThrowError('Base64 解码失败')
      spy.mockRestore()
    })

    it('round-trips arbitrary long and unicode inputs', () => {
      const inputs = [
        'Plain ASCII 12345',
        '含有中文的长文本'.repeat(1000),
        '👩🏽\u200D💻 Dev + Café ☕️'.repeat(500),
      ]
      for (const s of inputs) {
        const b64 = encodeBase64(s)
        expect(fromBase64(b64)).toBe(s)
        expect(decodeBase64(b64)).toBe(s)
      }
    })
  })
})
