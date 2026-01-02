import { ApiProperty } from '@nestjs/swagger'
import { ResponseCode, getResponseMessage } from '../enums/response-code.enum'

/**
 * 统一响应格式
 *
 * 注意：此类主要用于手动构建响应。
 * 大多数情况下，应使用 TransformInterceptor 自动包装响应。
 */
export class ResponseDto<T = any> {
  static ok<T>(data: T = null as T, message = 'Success'): ResponseDto<T> {
    return new ResponseDto(ResponseCode.SUCCESS, message, data)
  }

  static error<T>(
    code: ResponseCode = ResponseCode.ERROR,
    message?: string,
    data: T = null as T,
  ): ResponseDto<T> {
    return new ResponseDto(code, message || getResponseMessage(code), data)
  }

  @ApiProperty({ description: '状态码', example: 0 })
  code: number

  @ApiProperty({ description: '响应消息', example: 'Success' })
  message: string

  @ApiProperty({ description: '响应数据' })
  data: T

  @ApiProperty({ description: '时间戳', example: 1_640_000_000_000 })
  timestamp: number

  constructor(code: number, message: string, data: T) {
    this.code = code
    this.message = message
    this.data = data
    this.timestamp = Date.now()
  }
}
