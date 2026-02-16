/**
 * @seed/contracts — Layer 0 契约层
 *
 * 纯类型 + 纯数据定义，零运行时依赖（仅 zod）。
 * 所有语言/平台/应用层的项目都可安全引用这一层。
 *
 * 推荐通过子路径导入：
 *   import { userCreateSchema } from '@seed/contracts/schemas/user'
 *   import { BusinessError } from '@seed/contracts/errors'
 *   import { UserStatus } from '@seed/contracts/enums'
 *
 * 此 barrel export 仅用于便捷导入。
 */

// Schemas
export * from './schemas/index'

// Enums
export * from './enums/index'

// Errors
export {
  BusinessError,
  ValidationError,
  UnauthorizedError,
  ForbiddenError,
  NotFoundError,
  ConflictError,
  ResponseCode,
  mapCodeToHttpStatus,
} from './errors/business.error'
export type { ResponseCodeValue } from './errors/business.error'
