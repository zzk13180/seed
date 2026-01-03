/**
 * 防抖函数
 * 在连续调用时，只有最后一次调用后等待指定时间才会执行
 *
 * @param fn 需要防抖的函数
 * @param delay 延迟时间（毫秒）
 * @returns 防抖后的函数
 */
export function debounce<T extends (...args: unknown[]) => unknown>(
  fn: T,
  delay: number,
): (...args: Parameters<T>) => void {
  let timeoutId: ReturnType<typeof setTimeout> | null = null

  return (...args: Parameters<T>) => {
    if (timeoutId !== null) {
      clearTimeout(timeoutId)
    }

    timeoutId = setTimeout(() => {
      fn(...args)
      timeoutId = null
    }, delay)
  }
}

/**
 * 节流函数
 * 在指定时间内只执行一次
 *
 * @param fn 需要节流的函数
 * @param interval 间隔时间（毫秒）
 * @returns 节流后的函数
 */
export function throttle<T extends (...args: unknown[]) => unknown>(
  fn: T,
  interval: number,
): (...args: Parameters<T>) => void {
  let lastTime = 0

  return (...args: Parameters<T>) => {
    const now = Date.now()

    if (now - lastTime >= interval) {
      fn(...args)
      lastTime = now
    }
  }
}

/**
 * 深拷贝对象
 * 使用 structuredClone（如果可用），否则回退到 JSON 方法
 *
 * @param obj 需要拷贝的对象
 * @returns 深拷贝后的对象
 */
export function deepClone<T>(obj: T): T {
  if (obj === null || typeof obj !== 'object') {
    return obj
  }

  if (typeof structuredClone === 'function') {
    return structuredClone(obj)
  }

  return JSON.parse(JSON.stringify(obj)) as T
}

/**
 * 生成唯一 ID
 *
 * @param prefix 前缀
 * @returns 唯一 ID 字符串
 */
export function uniqueId(prefix = ''): string {
  const timestamp = Date.now().toString(36)
  const randomPart = Math.random().toString(36).slice(2, 9)
  return `${prefix}${timestamp}${randomPart}`
}

/**
 * 延迟指定时间
 *
 * @param ms 延迟时间（毫秒）
 * @returns Promise
 */
export function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms))
}

/**
 * 安全解析 JSON
 *
 * @param json JSON 字符串
 * @param fallback 解析失败时的默认值
 * @returns 解析后的对象或默认值
 */
export function safeJsonParse<T>(json: string, fallback: T): T {
  try {
    return JSON.parse(json) as T
  } catch {
    return fallback
  }
}

/**
 * 判断值是否为空（null、undefined、空字符串、空数组、空对象）
 */
export function isEmpty(value: unknown): boolean {
  if (value === null || value === undefined) {
    return true
  }

  if (typeof value === 'string') {
    return value.trim() === ''
  }

  if (Array.isArray(value)) {
    return value.length === 0
  }

  if (typeof value === 'object') {
    return Object.keys(value).length === 0
  }

  return false
}

/**
 * 判断值是否不为空
 */
export function isNotEmpty(value: unknown): boolean {
  return !isEmpty(value)
}

/**
 * 获取对象的嵌套属性值
 *
 * @param obj 目标对象
 * @param path 属性路径，如 'a.b.c' 或 'a[0].b'
 * @param defaultValue 默认值
 * @returns 属性值或默认值
 */
export function get<T = unknown>(
  obj: Record<string, unknown>,
  path: string,
  defaultValue?: T,
): T | undefined {
  const keys = path.replace(/\[(\d+)]/g, '.$1').split('.')
  let result: unknown = obj

  for (const key of keys) {
    if (result === null || result === undefined) {
      return defaultValue
    }
    result = (result as Record<string, unknown>)[key]
  }

  return (result === undefined ? defaultValue : result) as T | undefined
}

/**
 * 将对象转换为查询字符串
 *
 * @param params 参数对象
 * @returns 查询字符串（不带 ?）
 */
export function toQueryString(params: Record<string, unknown>): string {
  const pairs: string[] = []

  for (const [key, value] of Object.entries(params)) {
    if (value === undefined || value === null) {
      continue
    }

    if (Array.isArray(value)) {
      for (const item of value) {
        pairs.push(`${encodeURIComponent(key)}=${encodeURIComponent(String(item))}`)
      }
    } else {
      pairs.push(`${encodeURIComponent(key)}=${encodeURIComponent(String(value))}`)
    }
  }

  return pairs.join('&')
}

/**
 * 解析查询字符串为对象
 *
 * @param queryString 查询字符串（可以带 ? 也可以不带）
 * @returns 参数对象
 */
export function parseQueryString(queryString: string): Record<string, string | string[]> {
  const result: Record<string, string | string[]> = {}
  const query = queryString.startsWith('?') ? queryString.slice(1) : queryString

  if (!query) {
    return result
  }

  for (const pair of query.split('&')) {
    const [key, value] = pair.split('=').map(decodeURIComponent)
    if (!key) {
      continue
    }

    if (key in result) {
      const existing = result[key]!
      if (Array.isArray(existing)) {
        existing.push(value || '')
      } else {
        result[key] = [existing, value || '']
      }
    } else {
      result[key] = value || ''
    }
  }

  return result
}

/**
 * 格式化文件大小
 *
 * @param bytes 字节数
 * @param decimals 小数位数
 * @returns 格式化后的字符串
 */
export function formatFileSize(bytes: number, decimals = 2): string {
  if (bytes === 0) {
    return '0 Bytes'
  }

  const k = 1024
  const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB', 'PB']
  const i = Math.floor(Math.log(bytes) / Math.log(k))

  return `${Number.parseFloat((bytes / k ** i).toFixed(decimals))} ${sizes[i]}`
}

/**
 * 首字母大写
 */
export function capitalize(str: string): string {
  if (!str) {
    return ''
  }
  return str.charAt(0).toUpperCase() + str.slice(1)
}

/**
 * 驼峰转短横线
 */
export function kebabCase(str: string): string {
  return str.replace(/([a-z])([A-Z])/g, '$1-$2').toLowerCase()
}

/**
 * 短横线转驼峰
 */
export function camelCase(str: string): string {
  return str.replace(/-([a-z])/g, (_, letter: string) => letter.toUpperCase())
}
