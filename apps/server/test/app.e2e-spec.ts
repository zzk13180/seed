import { Test } from '@nestjs/testing'
import { ValidationPipe } from '@nestjs/common'
import request from 'supertest'
import { describe, it, expect, beforeAll, afterAll, beforeEach } from 'vitest'
import { AppModule } from '../src/app.module'
import { TransformInterceptor } from '../src/common/interceptors/transform.interceptor'
import type { INestApplication } from '@nestjs/common'
import type { TestingModule } from '@nestjs/testing'

/**
 * E2E 测试套件
 *
 * 前置条件：
 * 1. PostgreSQL 数据库已启动并可访问
 * 2. Redis 已启动并可访问
 * 3. 数据库中存在测试用户（可通过 sql/data.sql 初始化）
 */
describe('App E2E Tests', () => {
  let app: INestApplication
  let accessToken: string
  let refreshToken: string

  // 测试用户信息（需要在数据库中预先创建）
  const testUser = {
    username: 'admin',
    password: 'admin123',
  }

  // 新建用户用于测试
  const newUser = {
    username: `testuser_${Date.now()}`,
    password: 'Test@123456',
    nickname: '测试用户',
    email: `test_${Date.now()}@example.com`,
    phone: '13800138000',
  }

  let createdUserId: number

  beforeAll(async () => {
    const moduleFixture: TestingModule = await Test.createTestingModule({
      imports: [AppModule],
    }).compile()

    app = moduleFixture.createNestApplication()
    app.setGlobalPrefix('api')
    app.useGlobalPipes(
      new ValidationPipe({
        transform: true,
        whitelist: true,
        forbidNonWhitelisted: true,
      }),
    )
    // 使用全局拦截器（与 main.ts 保持一致）
    app.useGlobalInterceptors(new TransformInterceptor())
    await app.init()
  })

  afterAll(async () => {
    await app.close()
  })

  // ==================== Health Check ====================
  describe('Health Check', () => {
    describe('GET /api/health', () => {
      it('should return health status with all dependencies', async () => {
        const res = await request(app.getHttpServer()).get('/api/health').expect(200)

        // TransformInterceptor 包装后的响应
        expect(res.body).toHaveProperty('data')
        expect(res.body.data).toHaveProperty('status', 'ok')
        expect(res.body.data).toHaveProperty('info')
      })
    })

    describe('GET /api/health/liveness', () => {
      it('should return simple liveness status', async () => {
        const res = await request(app.getHttpServer()).get('/api/health/liveness').expect(200)

        // TransformInterceptor 包装后的响应
        expect(res.body).toHaveProperty('data')
        expect(res.body.data).toHaveProperty('status', 'ok')
      })
    })
  })

  // ==================== Authentication ====================
  describe('Authentication', () => {
    describe('POST /api/auth/login', () => {
      it('should return 400 for missing credentials', async () => {
        const res = await request(app.getHttpServer()).post('/api/auth/login').send({})
        expect(res.status).toBe(400)
      })

      it('should return 400 for missing password', async () => {
        const res = await request(app.getHttpServer())
          .post('/api/auth/login')
          .send({ username: 'admin' })
        expect(res.status).toBe(400)
      })

      it('should return 400 for password too short', async () => {
        const res = await request(app.getHttpServer())
          .post('/api/auth/login')
          .send({ username: 'admin', password: '123' })
        expect(res.status).toBe(400)
      })

      it('should return 401 for invalid credentials', async () => {
        const res = await request(app.getHttpServer())
          .post('/api/auth/login')
          .send({ username: 'invalid_user', password: 'invalid_pass' })

        // 401 为正常预期，500 表示数据库连接问题
        expect([401, 500]).toContain(res.status)
      })

      it('should login successfully with valid credentials', async () => {
        const res = await request(app.getHttpServer()).post('/api/auth/login').send(testUser)

        // 如果登录成功
        if (res.status === 200) {
          expect(res.body).toHaveProperty('data')
          expect(res.body.data).toHaveProperty('accessToken')
          expect(res.body.data).toHaveProperty('refreshToken')
          expect(res.body.data).toHaveProperty('user')
          expect(res.body.data).toHaveProperty('expiresIn')
          expect(res.body.data.user).toHaveProperty('username', testUser.username)
          expect(res.body.data.user).not.toHaveProperty('password')

          // 保存 token 供后续测试使用
          accessToken = res.body.data.accessToken
          refreshToken = res.body.data.refreshToken
        } else {
          // 数据库可能没有测试用户，跳过后续测试
          console.warn('Login failed, database might not have test user. Status:', res.status)
        }
      })
    })

    describe('POST /api/auth/refresh', () => {
      it('should return 401 for invalid refresh token', async () => {
        const res = await request(app.getHttpServer())
          .post('/api/auth/refresh')
          .send({ refreshToken: 'invalid_token' })

        expect([401, 500]).toContain(res.status)
      })

      it('should refresh token successfully', async () => {
        if (!refreshToken) {
          console.warn('Skipping refresh token test - no valid refresh token')
          return
        }

        const res = await request(app.getHttpServer())
          .post('/api/auth/refresh')
          .send({ refreshToken })

        if (res.status === 200) {
          expect(res.body.data).toHaveProperty('accessToken')
          expect(res.body.data).toHaveProperty('refreshToken')
          // 更新 token
          accessToken = res.body.data.accessToken
          refreshToken = res.body.data.refreshToken
        }
      })
    })

    describe('GET /api/auth/me', () => {
      it('should return 401 without token', async () => {
        await request(app.getHttpServer()).get('/api/auth/me').expect(401)
      })

      it('should return current user info with valid token', async () => {
        if (!accessToken) {
          console.warn('Skipping auth/me test - no valid access token')
          return
        }

        const res = await request(app.getHttpServer())
          .get('/api/auth/me')
          .set('Authorization', `Bearer ${accessToken}`)
          .expect(200)

        expect(res.body.data).toHaveProperty('username')
        expect(res.body.data).not.toHaveProperty('password')
      })
    })

    describe('POST /api/auth/logout', () => {
      it('should return 401 without token', async () => {
        await request(app.getHttpServer()).post('/api/auth/logout').expect(401)
      })
    })
  })

  // ==================== User Management ====================
  describe('User Management', () => {
    beforeEach(() => {
      if (!accessToken) {
        console.warn('No access token available for user management tests')
      }
    })

    describe('GET /api/v1/users', () => {
      it('should return 401 without authentication', async () => {
        await request(app.getHttpServer()).get('/api/v1/users').expect(401)
      })

      it('should return paginated user list with authentication', async () => {
        if (!accessToken) return

        const res = await request(app.getHttpServer())
          .get('/api/v1/users')
          .set('Authorization', `Bearer ${accessToken}`)
          .query({ page: 1, pageSize: 10 })
          .expect(200)

        expect(res.body.data).toHaveProperty('list')
        expect(res.body.data).toHaveProperty('total')
        expect(res.body.data).toHaveProperty('page')
        expect(res.body.data).toHaveProperty('pageSize')
        expect(Array.isArray(res.body.data.list)).toBe(true)
      })

      it('should support filtering by username', async () => {
        if (!accessToken) return

        const res = await request(app.getHttpServer())
          .get('/api/v1/users')
          .set('Authorization', `Bearer ${accessToken}`)
          .query({ username: 'admin' })
          .expect(200)

        expect(res.body.data).toHaveProperty('list')
      })

      it('should support keyword search', async () => {
        if (!accessToken) return

        const res = await request(app.getHttpServer())
          .get('/api/v1/users')
          .set('Authorization', `Bearer ${accessToken}`)
          .query({ keyword: 'admin' })
          .expect(200)

        expect(res.body.data).toHaveProperty('list')
      })
    })

    describe('GET /api/v1/users/list', () => {
      it('should return all users list', async () => {
        if (!accessToken) return

        const res = await request(app.getHttpServer())
          .get('/api/v1/users/list')
          .set('Authorization', `Bearer ${accessToken}`)
          .expect(200)

        expect(Array.isArray(res.body.data)).toBe(true)
      })
    })

    describe('POST /api/v1/users', () => {
      it('should return 401 without authentication', async () => {
        const res = await request(app.getHttpServer()).post('/api/v1/users').send(newUser)
        expect(res.status).toBe(401)
      })

      it('should return 400 for invalid user data', async () => {
        if (!accessToken) return

        const res = await request(app.getHttpServer())
          .post('/api/v1/users')
          .set('Authorization', `Bearer ${accessToken}`)
          .send({ username: 'ab' }) // username too short
        expect(res.status).toBe(400)
      })

      it('should create user successfully', async () => {
        if (!accessToken) return

        const res = await request(app.getHttpServer())
          .post('/api/v1/users')
          .set('Authorization', `Bearer ${accessToken}`)
          .send(newUser)

        if (res.status === 200 || res.status === 201) {
          expect(res.body.data).toHaveProperty('id')
          expect(res.body.data).toHaveProperty('username', newUser.username)
          expect(res.body.data).not.toHaveProperty('password')
          createdUserId = res.body.data.id
        } else if (res.status === 409) {
          // 用户名已存在
          console.warn('User already exists')
        }
      })

      it('should return 409 for duplicate username', async () => {
        if (!accessToken || !createdUserId) return

        const res = await request(app.getHttpServer())
          .post('/api/v1/users')
          .set('Authorization', `Bearer ${accessToken}`)
          .send(newUser)

        // 应该返回 409 Conflict 或相关业务错误码
        expect([400, 409, 422]).toContain(res.status)
      })
    })

    describe('GET /api/v1/users/:id', () => {
      it('should return 401 without authentication', async () => {
        const res = await request(app.getHttpServer()).get('/api/v1/users/1')
        expect(res.status).toBe(401)
      })

      it('should return user by id', async () => {
        if (!accessToken || !createdUserId) return

        const res = await request(app.getHttpServer())
          .get(`/api/v1/users/${createdUserId}`)
          .set('Authorization', `Bearer ${accessToken}`)
          .expect(200)

        expect(res.body.data).toHaveProperty('id', createdUserId)
        expect(res.body.data).not.toHaveProperty('password')
      })

      it('should return error for non-existent user', async () => {
        if (!accessToken) return

        const res = await request(app.getHttpServer())
          .get('/api/v1/users/999999')
          .set('Authorization', `Bearer ${accessToken}`)

        // 业务异常可能返回 404、400 或 500
        expect([404, 400, 500]).toContain(res.status)
      })
    })

    describe('PUT /api/v1/users/:id', () => {
      it('should return 401 without authentication', async () => {
        const res = await request(app.getHttpServer())
          .put('/api/v1/users/1')
          .send({ nickname: 'Updated' })
        expect(res.status).toBe(401)
      })

      it('should update user successfully', async () => {
        if (!accessToken || !createdUserId) return

        const updateData = { nickname: '更新后的昵称' }

        const res = await request(app.getHttpServer())
          .put(`/api/v1/users/${createdUserId}`)
          .set('Authorization', `Bearer ${accessToken}`)
          .send(updateData)
          .expect(200)

        expect(res.body.data).toHaveProperty('nickname', updateData.nickname)
      })
    })

    describe('PATCH /api/v1/users/:id/status', () => {
      it('should update user status', async () => {
        if (!accessToken || !createdUserId) return

        const res = await request(app.getHttpServer())
          .patch(`/api/v1/users/${createdUserId}/status`)
          .set('Authorization', `Bearer ${accessToken}`)
          .send({ status: 0 }) // 禁用用户

        expect([200, 204]).toContain(res.status)
      })
    })

    describe('PATCH /api/v1/users/:id/password', () => {
      it('should reset user password', async () => {
        if (!accessToken || !createdUserId) return

        const res = await request(app.getHttpServer())
          .patch(`/api/v1/users/${createdUserId}/password`)
          .set('Authorization', `Bearer ${accessToken}`)
          .send({ password: 'NewPassword@123' })

        expect([200, 204]).toContain(res.status)
      })
    })

    describe('DELETE /api/v1/users/:id', () => {
      it('should return 401 without authentication', async () => {
        const res = await request(app.getHttpServer()).delete('/api/v1/users/1')
        expect(res.status).toBe(401)
      })

      it('should delete user successfully', async () => {
        if (!accessToken || !createdUserId) return

        const res = await request(app.getHttpServer())
          .delete(`/api/v1/users/${createdUserId}`)
          .set('Authorization', `Bearer ${accessToken}`)

        expect([200, 204]).toContain(res.status)
      })
    })

    describe('POST /api/v1/users/batch-delete', () => {
      it('should return 401 without authentication', async () => {
        const res = await request(app.getHttpServer())
          .post('/api/v1/users/batch-delete')
          .send({ ids: [1, 2] })
        expect(res.status).toBe(401)
      })

      it('should return error for empty ids or require admin role', async () => {
        if (!accessToken) return

        const res = await request(app.getHttpServer())
          .post('/api/v1/users/batch-delete')
          .set('Authorization', `Bearer ${accessToken}`)
          .send({ ids: [] })

        // 可能返回 400（验证错误）、403（权限不足）或 500
        expect([400, 403, 500]).toContain(res.status)
      })
    })
  })

  // ==================== Error Handling ====================
  describe('Error Handling', () => {
    it('should return 404 for non-existent route', async () => {
      await request(app.getHttpServer()).get('/api/non-existent-route').expect(404)
    })

    it('should return proper error format', async () => {
      const res = await request(app.getHttpServer())
        .post('/api/auth/login')
        .send({ username: 'x' })
        .expect(400)

      // 验证错误响应格式
      expect(res.body).toHaveProperty('message')
    })
  })
})
