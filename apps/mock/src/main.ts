import { join } from 'node:path'
import { NestFactory } from '@nestjs/core'
import {
  Injectable,
  NestInterceptor,
  ExecutionContext,
  UseInterceptors,
  CallHandler,
  Module,
  Controller,
  Logger,
} from '@nestjs/common'
import { ServeStaticModule } from '@nestjs/serve-static'
import {
  loadMockConfig,
  MockConfig,
  RequestMethod,
  checkMockConfig,
} from './utils/mock.util'
import 'reflect-metadata'
import type { Observable } from 'rxjs'

const prefix = 'api-mock'
const requestMap = new Map()

@Injectable()
class TransformInterceptor<T> implements NestInterceptor<T, T> {
  intercept(context: ExecutionContext, next: CallHandler): Observable<T> {
    const ctx = context.switchToHttp()
    const request = ctx.getRequest()
    const regex = new RegExp(`/${prefix}/|/|(\\?.*)`, 'g')
    requestMap.set(request.url.replace(regex, ''), {
      query: request.query,
      body: request.body,
    })
    return next.handle()
  }
}

async function bootstrap() {
  const mockConfigs: MockConfig[] = await loadMockConfig()

  const registerMockEndpoints = <T extends { new (...args: any[]): {} }>(
    constructor: T,
  ) => {
    const { prototype } = constructor
    for (let i = 0, len = mockConfigs.length; i < len; i++) {
      const validConfigs: MockConfig[] = checkMockConfig(mockConfigs[i])
      if (!validConfigs.length) {
        continue
      }
      for (let j = 0, jlen = validConfigs.length; j < jlen; j++) {
        const config = validConfigs[j]
        const name = config.url.replace(/\//g, '')
        prototype[name] = (): string => {
          const req = requestMap.get(name)
          return JSON.stringify(config.response(req))
        }
        Reflect.defineMetadata('path', config.url, prototype[name])
        Reflect.defineMetadata(
          'method',
          RequestMethod[config.method?.toUpperCase() || 'GET'],
          prototype[name],
        )
      }
    }
    return class extends constructor {}
  }

  @UseInterceptors(TransformInterceptor)
  @registerMockEndpoints
  @Controller(prefix)
  class RootController {}

  @Module({
    imports: [
      ServeStaticModule.forRoot({
        rootPath: join(__dirname, '..', 'html'),
        exclude: [`/${prefix}*`],
      }),
    ],
    controllers: [RootController],
  })
  class RootModule {}
  const app = await NestFactory.create(RootModule)
  await app.listen(3333)
  Logger.log(`🔥 http://127.0.0.1:3333/${prefix}/hello`)
}

bootstrap()
