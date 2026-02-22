import { HTTPException } from 'hono/http-exception'
import { ZodError } from 'zod'
import {
  BusinessError,
  ValidationError,
  ResponseCode,
  mapCodeToHttpStatus,
} from '@seed/contracts/errors'
import type { Context } from 'hono'

/**
 * 全局错误处理器
 *
 * 统一处理 BusinessError / ZodError / HTTPException 等异常
 */
export function errorHandlerMiddleware(err: Error, c: Context): Response {
  const traceId = c.get('traceId') ?? null
  const path = c.req.path

  let httpStatus: number
  let code: number
  let message: string
  let errors: Record<string, string[]> | undefined

  if (err instanceof ValidationError) {
    httpStatus = mapCodeToHttpStatus(ResponseCode.VALIDATION_ERROR)
    code = err.code
    message = err.message
    errors = err.errors
  } else if (err instanceof BusinessError) {
    code = err.code
    message = err.message
    httpStatus = mapCodeToHttpStatus(code)
  } else if (err instanceof ZodError) {
    httpStatus = mapCodeToHttpStatus(ResponseCode.VALIDATION_ERROR)
    code = ResponseCode.VALIDATION_ERROR
    const fieldErrors: Record<string, string[]> = {}
    for (const issue of err.issues) {
      const field = issue.path.join('.')
      if (!fieldErrors[field]) fieldErrors[field] = []
      fieldErrors[field].push(issue.message)
    }
    message = '参数校验失败'
    errors = fieldErrors
  } else if (err instanceof HTTPException) {
    httpStatus = err.status
    code = err.status
    message = err.message
  } else {
    httpStatus = 500
    code = ResponseCode.INTERNAL_ERROR
    message = err instanceof Error ? err.message : 'Internal server error'
  }

  const level = httpStatus >= 500 ? 'error' : 'warn'
  console[level](
    `[${code}] ${c.req.method} ${path} - ${message}`,
    httpStatus >= 500 && err instanceof Error ? err.stack : undefined,
  )

  const body: Record<string, unknown> = {
    code,
    data: null,
    message,
    timestamp: Date.now(),
    path,
    traceId,
  }

  if (errors) {
    body.errors = errors
  }

  return c.json(body, httpStatus as any)
}
