import { Injectable, Logger, OnModuleInit } from '@nestjs/common'
import { EventEmitter2 } from '@nestjs/event-emitter'
import WebSocket from 'ws'

interface DialogueMessage {
  type: 'user_input' | 'text_update' | 'status_update'
  text: string
  status?: string
  conrtol_status?: string
}

@Injectable()
export class DialogueService implements OnModuleInit {
  private ws: WebSocket | null = null
  // 对话服务地址
  private reconnectAttempts = 0
  private readonly logger = new Logger(DialogueService.name)

  private readonly wsUrl = 'ws://127.0.0.1:8080/ws'

  private readonly maxReconnectAttempts = 10 // 最大重连次数
  private readonly reconnectInterval = 5000 // 重连间隔

  constructor(private readonly eventEmitter: EventEmitter2) {}

  onModuleInit() {
    this.connect()
  }

  sendText(text: string) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      const message = {
        type: 'send_text',
        text,
      }
      this.ws.send(JSON.stringify(message))
      this.logger.debug('已发送消息到对话服务:', message)
    } else {
      this.logger.warn('无法发送消息，对话服务未连接。')
    }
  }

  reconnect() {
    this.reconnectAttempts = 0
    this.connect()
  }

  private connect() {
    this.logger.log(`正在连接对话服务: ${this.wsUrl}...`)
    this.ws = new WebSocket(this.wsUrl)

    this.ws.on('open', () => {
      this.logger.log('已连接到对话服务。')
      this.reconnectAttempts = 0 // 重置重连计数
      // 连接成功后，发送一条消息测试对话
      // setTimeout(() => {
      //   this.sendText('你好')
      // }, 1000)
    })

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const message: DialogueMessage = JSON.parse(data.toString())
        this.logger.debug('收到对话服务消息:', message)

        // 为不同类型的消息触发独立的事件
        this.eventEmitter.emit(`dialogue.${message.type}`, message)
      } catch (error) {
        this.logger.error('解析对话服务消息失败', error)
      }
    })

    this.ws.on('error', error => {
      this.logger.error('对话服务 WebSocket 发生错误:', error.message)
    })

    this.ws.on('close', (code, reason) => {
      this.logger.warn(
        `对话服务连接已断开。代码: ${code}, 原因: ${reason.toString()}. 5秒后将重新连接...`,
      )
      if (this.reconnectAttempts < this.maxReconnectAttempts) {
        this.reconnectAttempts++
        this.logger.log(
          `${this.reconnectInterval / 1000}秒后将重新连接... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`,
        )
        setTimeout(() => this.connect(), this.reconnectInterval)
      } else {
        this.logger.error('已达到最大重连次数，停止重连。请检查对话服务是否可用。')
      }
    })
  }
}
