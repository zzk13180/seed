/**
 * 请求拦截器类型定义
 */
export type RequestInterceptor = (config: RequestConfig) => RequestConfig | Promise<RequestConfig>

/**
 * 响应拦截器类型定义
 */
export type ResponseInterceptor<T = unknown> = (
  response: T,
  config: RequestConfig,
) => T | Promise<T>

/**
 * 错误拦截器类型定义
 */
export type ErrorInterceptor = (error: HttpError, config: RequestConfig) => never | Promise<never>

/**
 * 请求配置接口
 */
export interface RequestConfig {
  endpoint: string
  method: string
  params?: unknown
  headers?: Record<string, string>
  timeout?: number
  retries?: number
  retryDelay?: number
}

/**
 * HTTP 错误类
 */
export class HttpError extends Error {
  constructor(
    message: string,
    public readonly status: number,
    public readonly data?: unknown,
    public readonly config?: RequestConfig,
  ) {
    super(message)
    this.name = 'HttpError'
    Object.setPrototypeOf(this, HttpError.prototype)
  }
}

/**
 * HTTP 客户端
 * 支持请求/响应拦截器、超时控制、自动重试
 */
export class HttpClient {
  private readonly requestInterceptors: RequestInterceptor[] = []
  private readonly responseInterceptors: ResponseInterceptor[] = []
  private readonly errorInterceptors: ErrorInterceptor[] = []

  basePath: string
  headers: Record<string, string>
  timeout: number
  retries: number
  retryDelay: number

  constructor(
    basePath: string,
    options: {
      headers?: Record<string, string>
      timeout?: number
      retries?: number
      retryDelay?: number
    } = {},
  ) {
    this.basePath = basePath
    this.headers = {
      'Content-Type': 'application/json',
      ...options.headers,
    }
    this.timeout = options.timeout ?? 30_000
    this.retries = options.retries ?? 0
    this.retryDelay = options.retryDelay ?? 1000
  }

  /**
   * 添加请求拦截器
   */
  addRequestInterceptor(interceptor: RequestInterceptor): () => void {
    this.requestInterceptors.push(interceptor)
    return () => {
      const index = this.requestInterceptors.indexOf(interceptor)
      if (index !== -1) {
        this.requestInterceptors.splice(index, 1)
      }
    }
  }

  /**
   * 添加响应拦截器
   */
  addResponseInterceptor<T>(interceptor: ResponseInterceptor<T>): () => void {
    this.responseInterceptors.push(interceptor as ResponseInterceptor)
    return () => {
      const index = this.responseInterceptors.indexOf(interceptor as ResponseInterceptor)
      if (index !== -1) {
        this.responseInterceptors.splice(index, 1)
      }
    }
  }

  /**
   * 添加错误拦截器
   */
  addErrorInterceptor(interceptor: ErrorInterceptor): () => void {
    this.errorInterceptors.push(interceptor)
    return () => {
      const index = this.errorInterceptors.indexOf(interceptor)
      if (index !== -1) {
        this.errorInterceptors.splice(index, 1)
      }
    }
  }

  setHeaders(headers: Record<string, string>): void {
    this.headers = {
      ...this.headers,
      ...headers,
    }
  }

  /**
   * 设置 Authorization 头
   */
  setAuthorization(token: string): void {
    this.headers.Authorization = `Bearer ${token}`
  }

  /**
   * 清除 Authorization 头
   */
  clearAuthorization(): void {
    delete this.headers.Authorization
  }

  private buildUrl(endpoint: string, params?: Record<string, unknown>): string {
    const url = new URL(`${this.basePath}${endpoint}`)
    if (params) {
      for (const [key, value] of Object.entries(params)) {
        if (value === undefined || value === null) {
          continue
        }

        if (Array.isArray(value)) {
          for (const v of value) {
            if (v !== undefined && v !== null) {
              url.searchParams.append(key, String(v))
            }
          }
        } else {
          url.searchParams.append(key, String(value))
        }
      }
    }
    return url.toString()
  }

  /**
   * 执行带超时和重试的请求
   */
  async request<T>(
    endpoint: string,
    method: string,
    params?: unknown,
    headers: Record<string, string> = {},
    options: { timeout?: number; retries?: number; retryDelay?: number } = {},
  ): Promise<T> {
    // 构建请求配置
    let config: RequestConfig = {
      endpoint,
      method,
      params,
      headers,
      timeout: options.timeout ?? this.timeout,
      retries: options.retries ?? this.retries,
      retryDelay: options.retryDelay ?? this.retryDelay,
    }

    // 执行请求拦截器
    for (const interceptor of this.requestInterceptors) {
      config = await interceptor(config)
    }

    const requestHeaders = {
      ...this.headers,
      ...config.headers,
    }

    const isGetMethod = config.method.toUpperCase() === 'GET'
    const url = isGetMethod
      ? this.buildUrl(config.endpoint, config.params as Record<string, unknown>)
      : `${this.basePath}${config.endpoint}`

    const requestOptions: RequestInit = {
      method: config.method,
      headers: requestHeaders,
      mode: 'cors',
    }

    // GET 请求不应该有 body
    if (!isGetMethod && config.params !== undefined && config.params !== null) {
      const isForm = typeof FormData !== 'undefined' && config.params instanceof FormData
      if (isForm) {
        // 移除 json 的 Content-Type 以避免 boundary 丢失
        delete (requestHeaders as Record<string, string>)['Content-Type']
        requestOptions.body = config.params as BodyInit
      } else {
        requestOptions.body = JSON.stringify(config.params)
      }
    }

    // 执行请求（带重试）
    let lastError: HttpError | undefined
    const maxAttempts = (config.retries ?? 0) + 1

    for (let attempt = 1; attempt <= maxAttempts; attempt++) {
      try {
        const response = await this.fetchWithTimeout(
          url,
          requestOptions,
          config.timeout ?? this.timeout,
        )

        // 204/205/304 或无内容
        if ([204, 205, 304].includes(response.status)) {
          return undefined as unknown as T
        }

        // 非 2xx 情况
        if (!response.ok) {
          const contentType = response.headers.get('content-type') || ''
          let errorData: unknown
          if (contentType.includes('application/json')) {
            errorData = await response.json()
          } else {
            errorData = await response.text()
          }

          const httpError = new HttpError(
            `HTTP Error ${response.status}`,
            response.status,
            errorData,
            config,
          )

          // 执行错误拦截器
          for (const interceptor of this.errorInterceptors) {
            await interceptor(httpError, config)
          }

          throw httpError
        }

        // 解析响应
        const contentType = response.headers.get('content-type') || ''
        let data: T
        if (contentType.includes('application/json')) {
          data = (await response.json()) as T
        } else {
          data = (await response.text()) as unknown as T
        }

        // 执行响应拦截器
        for (const interceptor of this.responseInterceptors) {
          data = (await interceptor(data, config)) as T
        }

        return data
      } catch (error) {
        lastError =
          error instanceof HttpError
            ? error
            : new HttpError(
                error instanceof Error ? error.message : 'Unknown error',
                0,
                error,
                config,
              )

        // 非网络错误或最后一次尝试，不再重试
        const isRetryable = lastError.status === 0 || lastError.status >= 500
        if (!isRetryable || attempt === maxAttempts) {
          break
        }

        // 等待后重试
        await this.delay(config.retryDelay ?? this.retryDelay)
      }
    }

    throw lastError
  }

  /**
   * 带超时的 fetch
   */
  private async fetchWithTimeout(
    url: string,
    options: RequestInit,
    timeout: number,
  ): Promise<Response> {
    const controller = new AbortController()
    const timeoutId = setTimeout(() => controller.abort(), timeout)

    try {
      const response = await fetch(url, {
        ...options,
        signal: controller.signal,
      })
      return response
    } catch (error) {
      if (error instanceof Error && error.name === 'AbortError') {
        throw new HttpError('Request timeout', 0)
      }
      throw error
    } finally {
      clearTimeout(timeoutId)
    }
  }

  /**
   * 延迟函数
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms))
  }

  async get<T>(
    endpoint: string,
    params?: Record<string, unknown>,
    headers?: Record<string, string>,
  ): Promise<T> {
    return this.request<T>(endpoint, 'GET', params, headers)
  }

  async post<T>(endpoint: string, params?: unknown, headers?: Record<string, string>): Promise<T> {
    return this.request<T>(endpoint, 'POST', params, headers)
  }

  async put<T>(endpoint: string, params?: unknown, headers?: Record<string, string>): Promise<T> {
    return this.request<T>(endpoint, 'PUT', params, headers)
  }

  async patch<T>(endpoint: string, params?: unknown, headers?: Record<string, string>): Promise<T> {
    return this.request<T>(endpoint, 'PATCH', params, headers)
  }

  async delete<T>(
    endpoint: string,
    params?: unknown,
    headers?: Record<string, string>,
  ): Promise<T> {
    return this.request<T>(endpoint, 'DELETE', params, headers)
  }

  async upload<T>(
    endpoint: string,
    formData: FormData,
    headers?: Record<string, string>,
  ): Promise<T> {
    return this.request<T>(endpoint, 'POST', formData, headers)
  }
}

/**
 * 创建 HTTP 客户端实例
 */
export function createHttpClient(
  basePath: string,
  options?: {
    headers?: Record<string, string>
    timeout?: number
    retries?: number
    retryDelay?: number
  },
): HttpClient {
  return new HttpClient(basePath, options)
}

// 默认实例（仅在浏览器环境使用）
export const $http =
  typeof import.meta !== 'undefined' && import.meta.env?.VITE_API_BASE_PATH
    ? new HttpClient(import.meta.env.VITE_API_BASE_PATH)
    : null
