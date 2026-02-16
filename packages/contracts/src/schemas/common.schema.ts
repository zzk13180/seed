import { z } from 'zod'

/**
 * 统一 API 响应 Schema 工厂
 *
 * 用法：apiResponseSchema(userVoSchema) → z.object({ code, data: UserVO, message, ... })
 */
export function apiResponseSchema<T extends z.ZodTypeAny>(dataSchema: T) {
  return z.object({
    code: z.number(),
    data: dataSchema,
    message: z.string(),
    timestamp: z.number(),
    traceId: z.string().nullable(),
  })
}

/**
 * 分页请求 Schema（通用基类）
 *
 * 各模块通过 .extend() 添加业务搜索字段
 */
export const pageRequestSchema = z.object({
  page: z.coerce.number().int().min(1).default(1),
  pageSize: z.coerce.number().int().min(1).max(100).default(10),
  orderBy: z.string().optional(),
  orderDirection: z.enum(['ASC', 'DESC']).default('DESC'),
  keyword: z.string().optional(),
})

/**
 * 分页响应 Schema 工厂
 */
export function pageResultSchema<T extends z.ZodTypeAny>(itemSchema: T) {
  return z.object({
    list: z.array(itemSchema),
    total: z.number(),
    page: z.number(),
    pageSize: z.number(),
    totalPages: z.number(),
    hasNext: z.boolean(),
    hasPrevious: z.boolean(),
  })
}

/**
 * 批量删除 Schema
 */
export const batchDeleteSchema = z.object({
  ids: z.array(z.number().int()).min(1).max(100),
})

// ────────────────────────────────────────────────────
// TypeScript 类型推导
// ────────────────────────────────────────────────────

export type ISODateString = string & { readonly __brand: 'ISODateString' }

export type ApiResponse<T = unknown> = {
  code: number
  data: T
  message: string
  timestamp: number
  traceId: string | null
}

export type PageRequest = z.infer<typeof pageRequestSchema>

export interface PageResult<T> {
  list: T[]
  total: number
  page: number
  pageSize: number
  totalPages: number
  hasNext: boolean
  hasPrevious: boolean
}

export type BatchDeleteParams = z.infer<typeof batchDeleteSchema>

/**
 * 构建分页结果
 */
export function createPageResult<T>(
  list: T[],
  total: number,
  page: number,
  pageSize: number,
): PageResult<T> {
  const totalPages = pageSize > 0 ? Math.ceil(total / pageSize) : 0
  return {
    list,
    total,
    page,
    pageSize,
    totalPages,
    hasNext: page < totalPages,
    hasPrevious: page > 1,
  }
}
