import { Module } from '@nestjs/common'
import { EventEmitterModule } from '@nestjs/event-emitter'
import { EventsGateway } from './events/events.gateway'
import { DialogueModule } from './dialogue/dialogue.module'

/**
 * 应用程序根模块
 *
 * 这是 NestJS 应用的入口模块，负责：
 * - 注册 WebSocket 事件网关
 * - 配置应用程序的依赖注入
 *
 * 当前功能：
 * - EventsGateway: 提供 WebSocket 通信服务
 */
@Module({
  imports: [
    EventEmitterModule.forRoot(), // 注册事件模块
    DialogueModule,
  ], // 导入的其他模块
  controllers: [], // HTTP 控制器（当前为空）
  providers: [EventsGateway], // 服务提供者，注册 WebSocket 网关
})
export class AppModule {}
