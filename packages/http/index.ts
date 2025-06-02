class HttpClient {
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
        if (params[key] !== undefined && params[key] !== null) {
          url.searchParams.append(key, String(params[key]))
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
    if (!isGetMethod && params) {
      requestOptions.body = JSON.stringify(params)
    }

    try {
      const response = await fetch(url, requestOptions)

      const data: unknown = await response.json()

      if (!response.ok) {
        throw data
      }

      return data as T
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
}

export const $http = new HttpClient(import.meta.env.VITE_API_BASE_PATH)
