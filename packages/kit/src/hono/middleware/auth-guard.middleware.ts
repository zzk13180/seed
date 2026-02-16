import { UnauthorizedError, ForbiddenError } from '@seed/contracts/errors'
import type { Context, Next } from 'hono'

/**
 * 认证守卫中间件
 *
 * 要求请求携带有效 session
 */
export async function authGuardMiddleware(c: Context, next: Next) {
  const session = c.get('session')
  if (!session?.user) {
    throw UnauthorizedError.tokenInvalid()
  }

  // 检查用户状态
  const user = session.user
  if (user.status === 0) {
    throw UnauthorizedError.accountDisabled()
  }

  await next()
}

/**
 * 角色守卫中间件工厂
 *
 * 检查用户是否拥有指定角色
 *
 * @example
 * ```ts
 * app.delete('/users/:id', rolesGuardMiddleware('admin'), handler)
 * ```
 */
export function rolesGuardMiddleware(...roles: string[]) {
  return async (c: Context, next: Next) => {
    const session = c.get('session')
    if (!session?.user) {
      throw ForbiddenError.insufficientPermissions()
    }

    const userRole = session.user.role ?? 'user'
    if (!roles.includes(userRole)) {
      throw ForbiddenError.roleRequired(roles.join(' 或 '))
    }

    await next()
  }
}

/**
 * 会话注入中间件
 *
 * 将 Better Auth session 注入到 Hono Context
 */
export function createSessionMiddleware(auth: {
  api: { getSession: (opts: { headers: Headers }) => Promise<any> }
}) {
  return async (c: Context, next: Next) => {
    const session = await auth.api.getSession({ headers: c.req.raw.headers })
    c.set('session', session)
    await next()
  }
}
