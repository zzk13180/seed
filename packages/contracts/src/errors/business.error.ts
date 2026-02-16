import { ResponseCode } from './response-code.enum'

/**
 * 业务异常基类
 *
 * 所有业务异常均应继承此类
 */
export class BusinessError extends Error {
  readonly code: number

  constructor(code: number, message: string) {
    super(message)
    this.name = 'BusinessError'
    this.code = code
  }
}

/**
 * 参数校验异常
 */
export class ValidationError extends BusinessError {
  readonly errors: Record<string, string[]>

  constructor(message: string, errors: Record<string, string[]>) {
    super(ResponseCode.VALIDATION_ERROR, message)
    this.name = 'ValidationError'
    this.errors = errors
  }
}

/**
 * 401 未认证异常
 */
export class UnauthorizedError extends BusinessError {
  static tokenInvalid(): UnauthorizedError {
    return new UnauthorizedError('无效的认证凭证')
  }

  static tokenExpired(): UnauthorizedError {
    return new UnauthorizedError('认证已过期，请重新登录')
  }

  static accountDisabled(): UnauthorizedError {
    return new UnauthorizedError('账号已被禁用')
  }

  constructor(message = '未认证或认证已过期') {
    super(ResponseCode.UNAUTHORIZED, message)
    this.name = 'UnauthorizedError'
  }
}

/**
 * 403 无权限异常
 */
export class ForbiddenError extends BusinessError {
  static insufficientPermissions(): ForbiddenError {
    return new ForbiddenError('权限不足')
  }

  static roleRequired(role: string): ForbiddenError {
    return new ForbiddenError(`需要角色: ${role}`)
  }

  constructor(message = '无权访问该资源') {
    super(ResponseCode.FORBIDDEN, message)
    this.name = 'ForbiddenError'
  }
}

/**
 * 404 资源不存在异常
 */
export class NotFoundError extends BusinessError {
  constructor(resource: string, id?: string | number) {
    const message = id ? `${resource} (${id}) 不存在` : `${resource} 不存在`
    super(ResponseCode.NOT_FOUND, message)
    this.name = 'NotFoundError'
  }
}

/**
 * 409 资源冲突异常
 */
export class ConflictError extends BusinessError {
  static emailExists(email: string): ConflictError {
    return new ConflictError(`邮箱 ${email} 已被使用`)
  }

  static usernameExists(username: string): ConflictError {
    return new ConflictError(`用户名 ${username} 已存在`)
  }

  constructor(message: string) {
    super(ResponseCode.CONFLICT, message)
    this.name = 'ConflictError'
  }
}

// Re-export response codes
export { ResponseCode, mapCodeToHttpStatus } from './response-code.enum'
export type { ResponseCodeValue } from './response-code.enum'
