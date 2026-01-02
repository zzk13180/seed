import { CallHandler, ExecutionContext, Injectable, NestInterceptor, Logger } from '@nestjs/common'
import { Observable, tap } from 'rxjs'

/**
 * 请求日志拦截器
 *
 * 记录每个 HTTP 请求的详细信息，类似于 Spring Boot 的 WebLogAspect
 * 包括：请求方法、URL、IP、User-Agent、请求体、响应时间等
 */
@Injectable()
export class LoggingInterceptor implements NestInterceptor {
  private readonly logger = new Logger('HTTP')

  intercept(context: ExecutionContext, next: CallHandler): Observable<unknown> {
    const request = context.switchToHttp().getRequest()
    const { method, url, body, headers } = request
    const ip = this.getClientIp(request)
    const userAgent = (headers['user-agent'] || '').slice(0, 80)
    const traceId = request.traceId || request.raw?.traceId || '-'
    const startTime = Date.now()

    // 请求开始日志
    this.logger.log(`[${traceId}] --> ${method} ${url} | IP: ${ip} | UA: ${userAgent}`)

    // 记录请求体（仅在非 GET 请求且有 body 时）
    if (method !== 'GET' && body && Object.keys(body).length > 0) {
      const sanitizedBody = this.sanitizeBody(body)
      this.logger.debug(`[${traceId}] Request Body: ${JSON.stringify(sanitizedBody).slice(0, 500)}`)
    }

    return next.handle().pipe(
      tap({
        next: () => {
          const costTime = Date.now() - startTime
          this.logger.log(`[${traceId}] <-- ${method} ${url} | ${costTime}ms`)
        },
        error: error => {
          const costTime = Date.now() - startTime
          this.logger.warn(
            `[${traceId}] <-- ${method} ${url} | ${costTime}ms | Error: ${error.message}`,
          )
        },
      }),
    )
  }

  /**
   * 获取客户端真实 IP
   * 支持代理服务器场景（X-Forwarded-For, X-Real-IP）
   */
  private getClientIp(request: any): string {
    const headers = request.headers || {}
    return (headers['x-forwarded-for']?.split(',')[0]?.trim() ||
      headers['x-real-ip'] ||
      request.ip ||
      request.connection?.remoteAddress ||
      'unknown') as string
  }

  /**
   * 脱敏请求体中的敏感字段
   * 防止密码等敏感信息出现在日志中
   */
  private sanitizeBody(body: Record<string, unknown>): Record<string, unknown> {
    const sensitiveFields = [
      'password',
      'newPassword',
      'oldPassword',
      'confirmPassword',
      'token',
      'refreshToken',
      'accessToken',
      'secret',
      'creditCard',
      'cvv',
    ]

    const sanitized = { ...body }
    for (const field of sensitiveFields) {
      if (field in sanitized) {
        sanitized[field] = '******'
      }
    }
    return sanitized
  }
}
