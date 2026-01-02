import { ResponseCode, getResponseMessage } from '../enums/response-code.enum'

/**
 * 业务异常基类
 *
 * 所有业务异常都应该继承此类，用于区分业务异常和系统异常
 */
export class BusinessException extends Error {
  /**
   * 创建通用业务异常
   */
  static of(code: ResponseCode, message?: string): BusinessException {
    return new BusinessException(code, message)
  }

  constructor(
    readonly code: ResponseCode,
    message?: string,
  ) {
    super(message || getResponseMessage(code))
    this.name = 'BusinessException'
    // 确保原型链正确
    Object.setPrototypeOf(this, BusinessException.prototype)
  }
}

/**
 * 数据不存在异常
 */
export class DataNotFoundException extends BusinessException {
  /**
   * 创建数据不存在异常
   */
  static create(resource: string, identifier: string | number): DataNotFoundException {
    return new DataNotFoundException(resource, identifier)
  }

  constructor(
    readonly resource: string,
    readonly identifier: string | number,
  ) {
    super(ResponseCode.DATA_NOT_FOUND, `${resource} with identifier '${identifier}' not found`)
    this.name = 'DataNotFoundException'
    Object.setPrototypeOf(this, DataNotFoundException.prototype)
  }
}

/**
 * 数据已存在异常
 */
export class DataAlreadyExistsException extends BusinessException {
  /**
   * 创建数据已存在异常
   */
  static create(resource: string, field: string, value: string): DataAlreadyExistsException {
    return new DataAlreadyExistsException(resource, field, value)
  }

  constructor(
    readonly resource: string,
    readonly field: string,
    readonly value: string,
  ) {
    super(ResponseCode.DATA_ALREADY_EXISTS, `${resource} with ${field}='${value}' already exists`)
    this.name = 'DataAlreadyExistsException'
    Object.setPrototypeOf(this, DataAlreadyExistsException.prototype)
  }
}

/**
 * 未授权异常
 */
export class UnauthorizedException extends BusinessException {
  /**
   * 无效凭证
   */
  static invalidCredentials(): UnauthorizedException {
    return new UnauthorizedException(getResponseMessage(ResponseCode.INVALID_CREDENTIALS))
  }

  /**
   * 令牌过期
   */
  static tokenExpired(): UnauthorizedException {
    return new UnauthorizedException(getResponseMessage(ResponseCode.TOKEN_EXPIRED))
  }

  /**
   * 无效令牌
   */
  static tokenInvalid(): UnauthorizedException {
    return new UnauthorizedException(getResponseMessage(ResponseCode.TOKEN_INVALID))
  }

  /**
   * 账号被禁用
   */
  static accountDisabled(): UnauthorizedException {
    return new UnauthorizedException(getResponseMessage(ResponseCode.ACCOUNT_DISABLED))
  }

  /**
   * 账号被锁定
   */
  static accountLocked(): UnauthorizedException {
    return new UnauthorizedException(getResponseMessage(ResponseCode.ACCOUNT_LOCKED))
  }

  /**
   * 登录尝试次数过多
   */
  static tooManyAttempts(minutes: number): UnauthorizedException {
    return new UnauthorizedException(`登录尝试次数过多，请 ${minutes} 分钟后再试`)
  }

  constructor(message?: string) {
    super(ResponseCode.UNAUTHORIZED, message)
    this.name = 'UnauthorizedException'
    Object.setPrototypeOf(this, UnauthorizedException.prototype)
  }
}

/**
 * 禁止访问异常
 */
export class ForbiddenException extends BusinessException {
  /**
   * 权限不足
   */
  static insufficientPermissions(): ForbiddenException {
    return new ForbiddenException(getResponseMessage(ResponseCode.INSUFFICIENT_PERMISSIONS))
  }

  /**
   * 需要特定角色
   */
  static roleRequired(role: string): ForbiddenException {
    return new ForbiddenException(`需要 ${role} 角色`)
  }

  constructor(message?: string) {
    super(ResponseCode.FORBIDDEN, message)
    this.name = 'ForbiddenException'
    Object.setPrototypeOf(this, ForbiddenException.prototype)
  }
}

/**
 * 参数验证异常
 */
export class ValidationException extends BusinessException {
  /**
   * 创建参数验证异常
   */
  static fromErrors(errors: Record<string, string[]>): ValidationException {
    const messages = Object.entries(errors)
      .map(([field, msgs]) => `${field}: ${msgs.join(', ')}`)
      .join('; ')
    return new ValidationException(errors, messages)
  }

  constructor(
    readonly errors: Record<string, string[]>,
    message?: string,
  ) {
    super(ResponseCode.VALIDATION_ERROR, message || '参数校验失败')
    this.name = 'ValidationException'
    Object.setPrototypeOf(this, ValidationException.prototype)
  }
}

/**
 * 用户相关异常
 */
export class UserException extends BusinessException {
  /**
   * 用户不存在
   */
  static notFound(identifier: string | number): UserException {
    return new UserException(
      ResponseCode.USER_NOT_FOUND,
      `User with identifier '${identifier}' not found`,
    )
  }

  /**
   * 用户名已存在
   */
  static usernameExists(username: string): UserException {
    return new UserException(
      ResponseCode.USERNAME_ALREADY_EXISTS,
      `Username '${username}' already exists`,
    )
  }

  /**
   * 邮箱已存在
   */
  static emailExists(email: string): UserException {
    return new UserException(ResponseCode.EMAIL_ALREADY_EXISTS, `Email '${email}' already exists`)
  }

  /**
   * 密码错误
   */
  static passwordIncorrect(): UserException {
    return new UserException(ResponseCode.PASSWORD_INCORRECT)
  }

  constructor(code: ResponseCode, message?: string) {
    super(code, message)
    this.name = 'UserException'
    Object.setPrototypeOf(this, UserException.prototype)
  }
}
