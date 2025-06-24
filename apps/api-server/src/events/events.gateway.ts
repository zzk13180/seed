import { Logger } from '@nestjs/common'
import { WebSocketGateway, OnGatewayConnection, OnGatewayDisconnect } from '@nestjs/websockets'
import { WebSocket } from 'ws'
import { OnEvent } from '@nestjs/event-emitter'
import { DialogueService } from '../dialogue/dialogue.service'
import { MessageType, RobotState } from './events.types'
import type {
  OutgoingMessage,
  InitMessage,
  AckMessage,
  RobotStateUpdateMessage,
} from './events.types'

@WebSocketGateway({ path: '/ws/robot' })
export class EventsGateway implements OnGatewayConnection, OnGatewayDisconnect {
  private readonly clients = new Set<WebSocket>()
  private readonly logger = new Logger(EventsGateway.name)

  constructor(private readonly dialogueService: DialogueService) {}

  handleConnection(client: WebSocket) {
    this.logger.log('[WS] 客户端已连接')
    this.clients.add(client)
    this.sendInit(client)
    this.setupClientListeners(client)
  }

  handleDisconnect(client: WebSocket) {
    this.clients.delete(client)
    this.logger.log('[WS] 客户端断开连接')
  }

  @OnEvent('dialogue.user_input')
  onUserInput(payload: { text: string }) {
    const msg: RobotStateUpdateMessage = {
      type: MessageType.ROBOT_STATE_UPDATE,
      data: { state: 'listening', expression: 'h0015', text: payload.text },
    }
    this.broadcast(msg)
  }

  @OnEvent('dialogue.text_update')
  onTextUpdate(payload: { text: string }) {
    const msg: RobotStateUpdateMessage = {
      type: MessageType.ROBOT_STATE_UPDATE,
      data: { state: 'responding', expression: 'h0006', text: payload.text },
    }
    this.broadcast(msg)
  }

  @OnEvent('dialogue.status_update')
  onStatusUpdate(payload: { status: string }) {
    this.logger.log(`[WS] 收到状态更新: ${payload.status}`)
    let msg: RobotStateUpdateMessage | null = null

    switch (payload.status) {
      case '待命': {
        msg = {
          type: MessageType.ROBOT_STATE_UPDATE,
          data: { state: 'idle', expression: 'h0063' },
        }
        break
      }

      case '聆听中...': {
        msg = {
          type: MessageType.ROBOT_STATE_UPDATE,
          data: { state: 'listening', expression: 'h0015' },
        }
        break
      }

      case '思考中...': {
        msg = {
          type: MessageType.ROBOT_STATE_UPDATE,
          data: { state: 'thinking', expression: 'h0242' },
        }
        break
      }

      case '说话中...': {
        msg = {
          type: MessageType.ROBOT_STATE_UPDATE,
          data: { state: 'responding', expression: 'h0006' },
        }
        break
      }

      default: {
        return
      }
    }

    this.broadcast(msg)
  }

  private sendInit(client: WebSocket) {
    const msg: InitMessage = { type: MessageType.INIT, data: { serverTime: Date.now() } }
    this.send(client, msg)
  }

  private setupClientListeners(client: WebSocket) {
    client.on('message', buffer => {
      const text = buffer.toString()
      this.logger.log(`[WS] 收到客户端: ${text}`)
      // 转发到 AI
      try {
        const obj = JSON.parse(text)
        if (obj.type === 'send_text') {
          this.dialogueService.sendText(obj.text)
        }
      } catch {}
      const ack: AckMessage = { type: MessageType.ACK, data: `已收到: ${text}` }
      this.send(client, ack)
    })
    client.on('error', err => this.handleDisconnect(client))
  }

  private send(client: WebSocket, msg: OutgoingMessage) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(msg))
    }
  }

  private broadcast(msg: OutgoingMessage) {
    for (const c of this.clients) this.send(c, msg)
  }
}
