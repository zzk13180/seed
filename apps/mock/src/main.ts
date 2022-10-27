import { join } from 'path'
import { NestFactory } from '@nestjs/core'
import { Module, Controller, Logger, Get } from '@nestjs/common'
import { ServeStaticModule } from '@nestjs/serve-static'
import {
  loadMockConfig,
  MockConfig,
  RequestMethod,
  checkMockConfig,
} from './utils/mock.util'
import 'reflect-metadata'
async function bootstrap() {
  const mockConfigArr: MockConfig[] = await loadMockConfig()

  @registerMockDatas
  @Controller(process.env.NODE_ENV === 'production' ? 'api' : 'api-mock')
  class RootController {
    @Get('test')
    async test(): Promise<string> {
      return new Promise((res) => {
        setTimeout(() => {
          res('test')
        }, 5000)
      })
    }
  }
  function registerMockDatas<T extends { new (...args: any[]): {} }>(constructor: T) {
    const prototype = constructor.prototype
    for (let i = 0, len = mockConfigArr.length; i < len; i++) {
      const items: MockConfig[] = checkMockConfig(mockConfigArr[i])
      if (!items.length) {
        continue
      }
      for (let j = 0, jlen = items.length; j < jlen; j++) {
        const item = items[j]
        const name = item.url.replace(/\//g, '')
        prototype[name] = (): string => JSON.stringify(item.response)
        Reflect.defineMetadata('path', item.url, prototype[name])
        Reflect.defineMetadata(
          'method',
          RequestMethod[item.method?.toUpperCase() || 'GET'],
          prototype[name],
        )
      }
    }
    return class extends constructor {}
  }

  @Module({
    imports: [
      ServeStaticModule.forRoot({
        rootPath: join(__dirname, '..', 'html'),
        exclude: ['/api*'],
      }),
    ],
    controllers: [RootController],
  })
  class RootModule {}
  const app = await NestFactory.create(RootModule)
  await app.listen(3333)
  Logger.log('http://127.0.0.1:3333/api-mock/test\nhttp://127.0.0.1:3333/')
}

bootstrap()
