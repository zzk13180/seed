/**
 * 统一响应接口
 */
export interface Response<T> {
  /** 响应状态码 */
  code: number
  /** 响应数据 */
  data: T
  /** 响应消息 */
  message: string
  /** 时间戳 */
  timestamp: number
  /** 请求追踪 ID */
  traceId?: string | null
}
