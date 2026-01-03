/**
 * ISO 8601 日期字符串类型
 * 用于 API 传输的日期格式
 */
export type ISODateString = string

/**
 * 统一 API 响应格式
 */
export interface ApiResponse<T = unknown> {
  /** 响应状态码 */
  code: number
  /** 响应数据 */
  data: T
  /** 响应消息 */
  message: string
  /** 时间戳 */
  timestamp: number
  /** 请求追踪 ID */
  traceId: string | null
}

/**
 * 分页请求参数
 */
export interface IPageRequest {
  /** 页码（从 1 开始） */
  page?: number
  /** 每页数量 */
  pageSize?: number
  /** 排序字段 */
  orderBy?: string
  /** 排序方向 */
  orderDirection?: 'ASC' | 'DESC'
  /** 关键字搜索 */
  keyword?: string
}

/**
 * 分页响应结果
 */
export interface IPageResult<T> {
  /** 数据列表 */
  list: T[]
  /** 总记录数 */
  total: number
  /** 当前页码 */
  page: number
  /** 每页数量 */
  pageSize: number
  /** 总页数 */
  totalPages: number
  /** 是否有下一页 */
  hasNext: boolean
  /** 是否有上一页 */
  hasPrevious: boolean
}

/**
 * 批量删除参数
 */
export interface IBatchDeleteParams {
  /** 要删除的 ID 列表 */
  ids: number[]
}
