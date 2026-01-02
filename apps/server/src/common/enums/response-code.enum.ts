/**
 * 统一响应码枚举
 *
 * 设计原则：
 * - 0: 成功
 * - 1-999: 通用错误
 * - 1000-1999: 用户相关错误
 * - 2000-2999: 认证相关错误
 * - 3000-3999: 权限相关错误
 * - 4000-4999: 业务相关错误
 * - 5000-5999: 系统相关错误
 */
export enum ResponseCode {
  // ==================== 成功 ====================
  SUCCESS = 0,

  // ==================== 通用错误 ====================
  ERROR = 1,
  BAD_REQUEST = 400,
  UNAUTHORIZED = 401,
  FORBIDDEN = 403,
  NOT_FOUND = 404,
  METHOD_NOT_ALLOWED = 405,
  CONFLICT = 409,
  INTERNAL_ERROR = 500,

  // ==================== 参数校验错误 1xxx ====================
  VALIDATION_ERROR = 1001,
  PARAM_MISSING = 1002,
  PARAM_INVALID = 1003,
  PARAM_TYPE_ERROR = 1004,

  // ==================== 认证相关错误 2xxx ====================
  INVALID_CREDENTIALS = 2001,
  TOKEN_EXPIRED = 2002,
  TOKEN_INVALID = 2003,
  REFRESH_TOKEN_EXPIRED = 2004,
  ACCOUNT_DISABLED = 2005,
  ACCOUNT_LOCKED = 2006,
  TOO_MANY_ATTEMPTS = 2007,

  // ==================== 权限相关错误 3xxx ====================
  ACCESS_DENIED = 3001,
  INSUFFICIENT_PERMISSIONS = 3002,
  ROLE_REQUIRED = 3003,

  // ==================== 业务相关错误 4xxx ====================
  DATA_NOT_FOUND = 4001,
  DATA_ALREADY_EXISTS = 4002,
  DATA_CONFLICT = 4003,
  OPERATION_FAILED = 4004,

  // 用户业务错误
  USER_NOT_FOUND = 4101,
  USER_ALREADY_EXISTS = 4102,
  USERNAME_ALREADY_EXISTS = 4103,
  EMAIL_ALREADY_EXISTS = 4104,
  PHONE_ALREADY_EXISTS = 4105,
  PASSWORD_INCORRECT = 4106,

  // ==================== 系统相关错误 5xxx ====================
  DATABASE_ERROR = 5001,
  REDIS_ERROR = 5002,
  EXTERNAL_SERVICE_ERROR = 5003,
}

/**
 * 响应码对应的默认消息
 */
export const ResponseMessages: Record<ResponseCode, string> = {
  // 成功
  [ResponseCode.SUCCESS]: '操作成功',

  // 通用错误
  [ResponseCode.ERROR]: '操作失败',
  [ResponseCode.BAD_REQUEST]: '请求参数错误',
  [ResponseCode.UNAUTHORIZED]: '未授权，请先登录',
  [ResponseCode.FORBIDDEN]: '禁止访问',
  [ResponseCode.NOT_FOUND]: '资源不存在',
  [ResponseCode.METHOD_NOT_ALLOWED]: '请求方法不允许',
  [ResponseCode.CONFLICT]: '数据冲突',
  [ResponseCode.INTERNAL_ERROR]: '服务器内部错误',

  // 参数校验错误
  [ResponseCode.VALIDATION_ERROR]: '参数校验失败',
  [ResponseCode.PARAM_MISSING]: '缺少必填参数',
  [ResponseCode.PARAM_INVALID]: '参数值无效',
  [ResponseCode.PARAM_TYPE_ERROR]: '参数类型错误',

  // 认证相关错误
  [ResponseCode.INVALID_CREDENTIALS]: '用户名或密码错误',
  [ResponseCode.TOKEN_EXPIRED]: '令牌已过期',
  [ResponseCode.TOKEN_INVALID]: '无效的令牌',
  [ResponseCode.REFRESH_TOKEN_EXPIRED]: '刷新令牌已过期',
  [ResponseCode.ACCOUNT_DISABLED]: '账号已被禁用',
  [ResponseCode.ACCOUNT_LOCKED]: '账号已被锁定',
  [ResponseCode.TOO_MANY_ATTEMPTS]: '登录尝试次数过多，请稍后再试',

  // 权限相关错误
  [ResponseCode.ACCESS_DENIED]: '访问被拒绝',
  [ResponseCode.INSUFFICIENT_PERMISSIONS]: '权限不足',
  [ResponseCode.ROLE_REQUIRED]: '需要特定角色',

  // 业务相关错误
  [ResponseCode.DATA_NOT_FOUND]: '数据不存在',
  [ResponseCode.DATA_ALREADY_EXISTS]: '数据已存在',
  [ResponseCode.DATA_CONFLICT]: '数据冲突',
  [ResponseCode.OPERATION_FAILED]: '操作失败',

  // 用户业务错误
  [ResponseCode.USER_NOT_FOUND]: '用户不存在',
  [ResponseCode.USER_ALREADY_EXISTS]: '用户已存在',
  [ResponseCode.USERNAME_ALREADY_EXISTS]: '用户名已存在',
  [ResponseCode.EMAIL_ALREADY_EXISTS]: '邮箱已存在',
  [ResponseCode.PHONE_ALREADY_EXISTS]: '手机号已存在',
  [ResponseCode.PASSWORD_INCORRECT]: '密码错误',

  // 系统相关错误
  [ResponseCode.DATABASE_ERROR]: '数据库错误',
  [ResponseCode.REDIS_ERROR]: '缓存服务错误',
  [ResponseCode.EXTERNAL_SERVICE_ERROR]: '外部服务调用失败',
}

/**
 * 获取响应码对应的消息
 */
export function getResponseMessage(code: ResponseCode): string {
  return ResponseMessages[code] || '未知错误'
}
