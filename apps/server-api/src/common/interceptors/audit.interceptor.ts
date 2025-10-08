import { Injectable, NestInterceptor, ExecutionContext, CallHandler } from '@nestjs/common'
import { Observable, Subscription } from 'rxjs'
import { AuditSubscriber } from '../subscribers/audit.subscriber'

/**
 * 审计拦截器
 *
 * 使用 AsyncLocalStorage 为每个请求创建独立的审计上下文
 * 确保在高并发环境下用户 ID 不会互相干扰
 */
@Injectable()
export class AuditInterceptor implements NestInterceptor {
  intercept(context: ExecutionContext, next: CallHandler): Observable<any> {
    const request = context.switchToHttp().getRequest()
    const user = request.user
    const userId = user?.id || user?.sub || null

    // 创建新的 Observable，在 AsyncLocalStorage 上下文中订阅原始 Observable
    return new Observable(subscriber => {
      let subscription: Subscription | undefined

      // 使用 AsyncLocalStorage.run() 包装整个订阅过程
      // 确保在异步操作中也能获取正确的用户 ID
      AuditSubscriber.run(userId, () => {
        subscription = next.handle().subscribe({
          next: value => subscriber.next(value),
          error: err => subscriber.error(err),
          complete: () => subscriber.complete(),
        })
      })

      // 返回清理函数，确保订阅能被正确取消
      return () => subscription?.unsubscribe()
    })
  }
}
