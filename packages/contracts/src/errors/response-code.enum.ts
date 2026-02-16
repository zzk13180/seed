/**
 * 统一响应码枚举
 *
 * HTTP 语义状态码 + 业务码分离
 */
export const ResponseCode = {
  /** 成功 */
  SUCCESS: 200,

  /** 客户端错误 */
  BAD_REQUEST: 400,
  UNAUTHORIZED: 401,
  FORBIDDEN: 403,
  NOT_FOUND: 404,
  CONFLICT: 409,
  VALIDATION_ERROR: 422,
  TOO_MANY_REQUESTS: 429,

  /** 服务端错误 */
  INTERNAL_ERROR: 500,
  SERVICE_UNAVAILABLE: 503,
} as const

export type ResponseCodeValue = (typeof ResponseCode)[keyof typeof ResponseCode]

/**
 * 响应码 → HTTP 状态码映射
 */
export function mapCodeToHttpStatus(code: number): number {
  if (code >= 200 && code < 600) return code
  return 500
}
