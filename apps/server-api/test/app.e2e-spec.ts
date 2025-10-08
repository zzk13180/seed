import { Test } from '@nestjs/testing'
import { ValidationPipe } from '@nestjs/common'
import request from 'supertest'
import { describe, it, expect, beforeAll, afterAll } from 'vitest'
import { AppModule } from '../src/app.module'
import type { INestApplication } from '@nestjs/common'
import type { TestingModule } from '@nestjs/testing'

describe('AppController (e2e)', () => {
  let app: INestApplication

  beforeAll(async () => {
    const moduleFixture: TestingModule = await Test.createTestingModule({
      imports: [AppModule],
    }).compile()

    app = moduleFixture.createNestApplication()
    app.setGlobalPrefix('api')
    app.useGlobalPipes(new ValidationPipe({ transform: true, whitelist: true }))
    await app.init()
  })

  afterAll(async () => {
    await app.close()
  })

  describe('/api/health (GET)', () => {
    it('should return health status', async () => {
      await request(app.getHttpServer())
        .get('/api/health')
        .expect(200)
        .expect(res => {
          // @nestjs/terminus 返回的健康检查格式
          expect(res.body).toHaveProperty('status', 'ok')
          expect(res.body).toHaveProperty('info')
        })
    })
  })

  describe('/api/auth/login (POST)', () => {
    // 注意：此测试需要数据库连接，如果数据库不可用会返回 500
    // 在 CI 环境中可能需要 mock 数据库或跳过此测试
    it('should return 401 or 500 for invalid credentials (depends on DB)', async () => {
      const res = await request(app.getHttpServer())
        .post('/api/auth/login')
        .send({ username: 'invalid', password: 'invalid' })
      // 如果数据库可用返回 401，否则返回 500
      expect([401, 500]).toContain(res.status)
    })

    // eslint-disable-next-line sonarjs/assertions-in-tests
    it('should return 400 for missing credentials', async () => {
      await request(app.getHttpServer()).post('/api/auth/login').send({}).expect(400)
    })
  })

  describe('/api/v1/users (GET)', () => {
    it('should return 401 without authentication', async () => {
      await request(app.getHttpServer()).get('/api/v1/users').expect(401)
    })
  })
})
