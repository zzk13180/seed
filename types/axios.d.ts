export {}
declare module 'axios' {
  interface AxiosRequestConfig {
    pendingKey?: symbol
    isJsonp?: boolean
  }
}
