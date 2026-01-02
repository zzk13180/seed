import {
  ExceptionFilter,
  Catch,
  ArgumentsHost,
  HttpException,
  HttpStatus,
  Logger,
} from '@nestjs/common'
import { HttpAdapterHost } from '@nestjs/core'
import { BusinessException, ValidationException } from '../exceptions/business.exception'
import { ResponseCode } from '../enums/response-code.enum'

/**
 * 全局异常过滤器
 * 捕获所有异常并返回统一格式的响应
 */
@Catch()
export class AllExceptionsFilter implements ExceptionFilter {
  private readonly logger = new Logger(AllExceptionsFilter.name)

  constructor(private readonly httpAdapterHost: HttpAdapterHost) {}

  catch(exception: unknown, host: ArgumentsHost): void {
    const { httpAdapter } = this.httpAdapterHost
    const ctx = host.switchToHttp()
    const request = ctx.getRequest()

    let httpStatus: number
    let code: number
    let message: string
    let errors: Record<string, string[]> | undefined

    // 处理业务异常
    if (exception instanceof BusinessException) {
      code = exception.code
      message = exception.message
      // 根据业务错误码映射 HTTP 状态码
      httpStatus = this.mapBusinessCodeToHttpStatus(exception.code)

      // 处理 ValidationException 的详细错误信息
      if (exception instanceof ValidationException) {
        errors = exception.errors
      }
    }
    // 处理 HTTP 异常
    else if (exception instanceof HttpException) {
      httpStatus = exception.getStatus()
      code = httpStatus
      const response = exception.getResponse()
      message =
        typeof response === 'string' ? response : (response as any).message || exception.message
      // 处理 class-validator 的验证错误数组
      if (Array.isArray(message)) {
        message = message.join('; ')
      }
    }
    // 处理其他异常
    else {
      httpStatus = HttpStatus.INTERNAL_SERVER_ERROR
      code = ResponseCode.INTERNAL_ERROR
      message = exception instanceof Error ? exception.message : 'Internal server error'
    }

    // 统一响应格式
    const responseBody: Record<string, unknown> = {
      code,
      data: null,
      message,
      timestamp: Date.now(),
      path: httpAdapter.getRequestUrl(request),
      traceId: request.traceId || null,
    }

    // 如果有详细的字段验证错误，添加到响应中
    if (errors) {
      responseBody.errors = errors
    }

    // 记录日志
    const logLevel = httpStatus >= 500 ? 'error' : 'warn'
    this.logger[logLevel](
      `[${String(code)}] ${request.method} ${String(responseBody.path)} - ${message}`,
      exception instanceof Error ? exception.stack : undefined,
    )

    httpAdapter.reply(ctx.getResponse(), responseBody, httpStatus)
  }

  /**
   * 将业务错误码映射到 HTTP 状态码
   */
  private mapBusinessCodeToHttpStatus(code: ResponseCode): number {
    // 直接的 HTTP 状态码
    if ((code as number) >= 400 && (code as number) < 600) {
      return code as unknown as number
    }

    // 业务错误码映射
    const mapping: Record<number, number> = {
      // 参数校验错误 -> 400
      [ResponseCode.VALIDATION_ERROR]: HttpStatus.BAD_REQUEST,
      [ResponseCode.PARAM_MISSING]: HttpStatus.BAD_REQUEST,
      [ResponseCode.PARAM_INVALID]: HttpStatus.BAD_REQUEST,
      [ResponseCode.PARAM_TYPE_ERROR]: HttpStatus.BAD_REQUEST,

      // 认证错误 -> 401
      [ResponseCode.INVALID_CREDENTIALS]: HttpStatus.UNAUTHORIZED,
      [ResponseCode.TOKEN_EXPIRED]: HttpStatus.UNAUTHORIZED,
      [ResponseCode.TOKEN_INVALID]: HttpStatus.UNAUTHORIZED,
      [ResponseCode.REFRESH_TOKEN_EXPIRED]: HttpStatus.UNAUTHORIZED,
      [ResponseCode.ACCOUNT_DISABLED]: HttpStatus.UNAUTHORIZED,
      [ResponseCode.ACCOUNT_LOCKED]: HttpStatus.UNAUTHORIZED,
      [ResponseCode.TOO_MANY_ATTEMPTS]: HttpStatus.TOO_MANY_REQUESTS,

      // 权限错误 -> 403
      [ResponseCode.ACCESS_DENIED]: HttpStatus.FORBIDDEN,
      [ResponseCode.INSUFFICIENT_PERMISSIONS]: HttpStatus.FORBIDDEN,
      [ResponseCode.ROLE_REQUIRED]: HttpStatus.FORBIDDEN,

      // 数据不存在 -> 404
      [ResponseCode.DATA_NOT_FOUND]: HttpStatus.NOT_FOUND,
      [ResponseCode.USER_NOT_FOUND]: HttpStatus.NOT_FOUND,

      // 数据冲突 -> 409
      [ResponseCode.DATA_ALREADY_EXISTS]: HttpStatus.CONFLICT,
      [ResponseCode.DATA_CONFLICT]: HttpStatus.CONFLICT,
      [ResponseCode.USER_ALREADY_EXISTS]: HttpStatus.CONFLICT,
      [ResponseCode.USERNAME_ALREADY_EXISTS]: HttpStatus.CONFLICT,
      [ResponseCode.EMAIL_ALREADY_EXISTS]: HttpStatus.CONFLICT,
      [ResponseCode.PHONE_ALREADY_EXISTS]: HttpStatus.CONFLICT,

      // 系统错误 -> 500
      [ResponseCode.DATABASE_ERROR]: HttpStatus.INTERNAL_SERVER_ERROR,
      [ResponseCode.REDIS_ERROR]: HttpStatus.INTERNAL_SERVER_ERROR,
      [ResponseCode.EXTERNAL_SERVICE_ERROR]: HttpStatus.BAD_GATEWAY,
    }

    // 未知业务码默认返回 500，而非 200
    return mapping[code] || HttpStatus.INTERNAL_SERVER_ERROR
  }
}
