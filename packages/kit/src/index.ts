/**
 * @seed/kit — Layer 1 基础设施层
 *
 * 运行时基础设施代码，无业务逻辑。按技术栈分 subpath，项目按需引入。
 *
 * 服务端:
 *   import { createHonoApp } from '@seed/kit/hono/app-factory'
 *   import { createAuth } from '@seed/kit/auth/server'
 *
 * 前端:
 *   import { BaseController } from '@seed/kit/frontend'
 *
 * 工具:
 *   import { formatDate } from '@seed/kit/utils'
 *
 * 类型/Schema/Enum/Error 已迁移至 @seed/contracts (Layer 0):
 *   import { UserVO, BusinessError, UserStatus } from '@seed/contracts'
 */

// 此包不再有 barrel export — 请通过 subpath 导入
export {}
