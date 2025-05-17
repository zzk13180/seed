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

  async request<T>(
    endpoint: string,
    method: string,
    params: any,
    headers: Record<string, string> = {},
  ): Promise<T> {
    const url = `${this.basePath}${endpoint}`
    const requestHeaders = {
      ...this.headers,
      ...headers,
    }

    try {
      const response = await fetch(url, {
        method,
        headers: requestHeaders,
        body: JSON.stringify(params),
        mode: 'cors',
      })

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
}

export const $http = new HttpClient(import.meta.env.VITE_API_BASE_PATH)
