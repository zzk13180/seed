import { Hono } from 'hono'

/**
 * 创建认证路由
 *
 * Better Auth 路由代理 — 所有 `/api/auth/**` 请求委托给 Better Auth 处理
 *
 * @param auth - Better Auth 实例（提供 .handler() 方法）
 */
export function createAuthRoutes(auth: {
  handler: (req: Request) => Response | Promise<Response>
}) {
  return new Hono().on(['GET', 'POST'], '/*', c => {
    return auth.handler(c.req.raw)
  })
}
