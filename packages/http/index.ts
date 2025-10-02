export class HttpClient {
  basePath: string
  headers: Record<string, string>

  constructor(basePath: string, headers: Record<string, string> = {}) {
    this.basePath = basePath
    this.headers = {
      'Content-Type': 'application/json',
      ...headers,
    }
  }

  setHeaders(headers: Record<string, string>) {
    this.headers = {
      ...this.headers,
      ...headers,
    }
  }

  private buildUrl(endpoint: string, params?: Record<string, any>): string {
    const url = new URL(`${this.basePath}${endpoint}`)
    if (params) {
      Object.keys(params).forEach(key => {
        const value = (params as any)[key]
        if (value === undefined || value === null) return

        if (Array.isArray(value)) {
          value.forEach(v => {
            if (v !== undefined && v !== null) url.searchParams.append(key, String(v))
          })
        } else {
          url.searchParams.append(key, String(value))
        }
      })
    }
    return url.toString()
  }

  async request<T>(
    endpoint: string,
    method: string,
    params?: any,
    headers: Record<string, string> = {},
  ): Promise<T> {
    const requestHeaders = {
      ...this.headers,
      ...headers,
    }

    const isGetMethod = method.toUpperCase() === 'GET'
    const url = isGetMethod ? this.buildUrl(endpoint, params) : `${this.basePath}${endpoint}`

    const requestOptions: RequestInit = {
      method,
      headers: requestHeaders,
      mode: 'cors',
    }

    // GET 请求不应该有 body
    if (!isGetMethod && params !== undefined && params !== null) {
      // FormData 上传时，由浏览器自动设置 Content-Type，不能手动设置
      const isForm = typeof FormData !== 'undefined' && params instanceof FormData
      if (isForm) {
        // 移除 json 的 Content-Type 以避免 boundary 丢失
        if ('Content-Type' in requestHeaders) {
          delete (requestHeaders as any)['Content-Type']
        }
        requestOptions.body = params as BodyInit
      } else {
        requestOptions.body = JSON.stringify(params)
      }
    }

    try {
      const response = await fetch(url, requestOptions)
      // 204/205/304 或无内容
      if ([204, 205, 304].includes(response.status)) {
        return undefined as unknown as T
      }

      // 非 2xx 情况，尽量解析错误详情
      if (!response.ok) {
        const contentType = response.headers.get('content-type') || ''
        try {
          if (contentType.includes('application/json')) {
            const errData = await response.json()
            throw errData
          } else {
            const errText = await response.text()
            throw errText
          }
        } catch (e) {
          throw e
        }
      }

      const contentType = response.headers.get('content-type') || ''
      if (contentType.includes('application/json')) {
        const data: unknown = await response.json()
        return data as T
      }

      // 回退为文本
      const textData = await response.text()
      return textData as unknown as T
    } catch (error) {
      console.error('API请求失败:', error)
      throw error
    }
  }

  async get<T>(
    endpoint: string,
    params?: Record<string, any>,
    headers?: Record<string, string>,
  ): Promise<T> {
    return this.request<T>(endpoint, 'GET', params, headers)
  }

  async post<T>(endpoint: string, params?: any, headers?: Record<string, string>): Promise<T> {
    return this.request<T>(endpoint, 'POST', params, headers)
  }

  async put<T>(endpoint: string, params?: any, headers?: Record<string, string>): Promise<T> {
    return this.request<T>(endpoint, 'PUT', params, headers)
  }

  async delete<T>(endpoint: string, params?: any, headers?: Record<string, string>): Promise<T> {
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

export const $http = new HttpClient(import.meta.env.VITE_API_BASE_PATH)
