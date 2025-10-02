import { join } from 'node:path'
import { Module } from '@nestjs/common'
import { ServeStaticModule } from '@nestjs/serve-static'
import { VersionModule } from './version/version.module'

@Module({
  imports: [
    ServeStaticModule.forRoot({
      rootPath: join(process.cwd(), 'public'),
      serveRoot: '/',
      exclude: ['/api*', '/appimage*', '/versions*'],
    }),
    VersionModule,
  ],
  providers: [],
  exports: [],
})
export class AppModule {}
