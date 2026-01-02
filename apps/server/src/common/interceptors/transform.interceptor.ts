import { Injectable, NestInterceptor, ExecutionContext, CallHandler } from '@nestjs/common'
import { Observable } from 'rxjs'
import { map } from 'rxjs/operators'
import { Response } from '../interfaces/response.interface'
import { ResponseCode } from '../enums/response-code.enum'

@Injectable()
export class TransformInterceptor<T> implements NestInterceptor<T, Response<T>> {
  intercept(context: ExecutionContext, next: CallHandler): Observable<Response<T>> {
    const request = context.switchToHttp().getRequest()
    const traceId = request.traceId || request.raw?.traceId || null

    return next.handle().pipe(
      map(data => ({
        code: ResponseCode.SUCCESS,
        data,
        message: 'Success',
        timestamp: Date.now(),
        traceId,
      })),
    )
  }
}
