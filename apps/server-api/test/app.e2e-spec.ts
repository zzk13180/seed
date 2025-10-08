import { Test } from '@nestjs/testing'
import { ValidationPipe } from '@nestjs/common'
import * as request from 'supertest'
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
          expect(res.body).toHaveProperty('status', 'ok')
          expect(res.body).toHaveProperty('timestamp')
        })
    })
  })

  describe('/api/auth/login (POST)', () => {
    // eslint-disable-next-line sonarjs/assertions-in-tests
    it('should return 401 for invalid credentials', async () => {
      await request(app.getHttpServer())
        .post('/api/auth/login')
        .send({ username: 'invalid', password: 'invalid' })
        .expect(401)
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
