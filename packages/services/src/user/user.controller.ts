import { Hono } from 'hono'
import { zValidator } from '@hono/zod-validator'
import { authGuardMiddleware, rolesGuardMiddleware } from '@seed/kit/hono/middleware'
import {
  userQuerySchema,
  userUpdateSchema,
  updateStatusSchema,
  resetPasswordSchema,
  batchDeleteUserSchema,
  idParamSchema,
} from '@seed/contracts/schemas/user'
import { UserService, type HashPasswordFn } from './user.service'
import type { AnyDatabase } from '@seed/db'

export interface UserRoutesOptions {
  /** 密码哈希函数（可选，默认使用 Bun.password.hash） */
  hashPassword?: HashPasswordFn
}

/**
 * 创建用户管理路由
 *
 * 标准 CRUD 模块: 分页查询 / 详情 / 更新 / 删除 / 批量删除 / 状态更新 / 密码重置
 * 全部路由需要认证，管理操作需要 admin 角色
 *
 * @param db - 数据库实例（Neon HTTP 或 node-postgres 均可）
 * @param options - 可选配置（hashPassword 等）
 */
export function createUserRoutes(db: AnyDatabase, options?: UserRoutesOptions) {
  const userService = new UserService(db, options?.hashPassword)

  return (
    new Hono()
      // 全部需要认证
      .use('*', authGuardMiddleware)

      /**
       * GET / — 分页查询用户
       */
      .get('/', zValidator('query', userQuerySchema), async c => {
        const query = c.req.valid('query')
        const result = await userService.findPage(query)
        return c.json(result)
      })

      /**
       * GET /list — 查询所有用户
       */
      .get('/list', async c => {
        const list = await userService.findAll()
        return c.json(list)
      })

      /**
       * GET /:id — 根据 ID 查询用户
       */
      .get('/:id', zValidator('param', idParamSchema), async c => {
        const { id } = c.req.valid('param')
        const user = await userService.findById(id)
        return c.json(user)
      })

      /**
       * PUT /:id — 更新用户
       */
      .put(
        '/:id',
        zValidator('param', idParamSchema),
        zValidator('json', userUpdateSchema),
        async c => {
          const { id } = c.req.valid('param')
          const dto = c.req.valid('json')
          const user = await userService.update(id, dto)
          return c.json(user)
        },
      )

      /**
       * DELETE /:id — 删除用户（软删除）— 需要 admin 角色
       */
      .delete(
        '/:id',
        rolesGuardMiddleware('admin'),
        zValidator('param', idParamSchema),
        async c => {
          const { id } = c.req.valid('param')
          await userService.delete(id)
          return c.body(null, 204)
        },
      )

      /**
       * POST /batch-delete — 批量删除用户 — 需要 admin 角色
       */
      .post(
        '/batch-delete',
        rolesGuardMiddleware('admin'),
        zValidator('json', batchDeleteUserSchema),
        async c => {
          const { ids } = c.req.valid('json')
          await userService.batchDelete(ids)
          return c.body(null, 204)
        },
      )

      /**
       * PATCH /:id/status — 更新用户状态 — 需要 admin 角色
       */
      .patch(
        '/:id/status',
        rolesGuardMiddleware('admin'),
        zValidator('param', idParamSchema),
        zValidator('json', updateStatusSchema),
        async c => {
          const { id } = c.req.valid('param')
          const { status } = c.req.valid('json')
          await userService.updateStatus(id, status)
          return c.body(null, 204)
        },
      )

      /**
       * PATCH /:id/password — 重置用户密码 — 需要 admin 角色
       */
      .patch(
        '/:id/password',
        rolesGuardMiddleware('admin'),
        zValidator('param', idParamSchema),
        zValidator('json', resetPasswordSchema),
        async c => {
          const { id } = c.req.valid('param')
          const { password } = c.req.valid('json')
          await userService.resetPassword(id, password)
          return c.body(null, 204)
        },
      )
  )
}
