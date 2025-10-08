import { $http } from '@seed/http'
import type { PanelApiService, PanelWsService, RobotInfo } from './panel.types'

/**
 * HTTP Panel API 服务实现
 */
export class HttpPanelApiService implements PanelApiService {
  async fetchRobotInfo(): Promise<RobotInfo> {
    try {
      const response = await $http.get<{ data: RobotInfo }>('/robot/info')
      return response.data
    } catch {
      // 返回模拟数据（开发阶段）
      return {
        robotId: 'robot-001',
        name: 'Test Robot',
        status: 'online',
        batteryLevel: 85,
        lastUpdated: new Date().toISOString(),
      }
    }
  }

  async updateRobotInfo(info: Partial<RobotInfo>): Promise<RobotInfo> {
    const response = await $http.patch<{ data: RobotInfo }>('/robot/info', info)
    return response.data
  }
}

/**
 * Mock Panel API 服务（用于开发和测试）
 */
export class MockPanelApiService implements PanelApiService {
  private robotInfo: RobotInfo = {
    robotId: 'robot-001',
    name: 'Test Robot',
    status: 'online',
    batteryLevel: 85,
    lastUpdated: new Date().toISOString(),
  }

  async fetchRobotInfo(): Promise<RobotInfo> {
    // 模拟网络延迟
    await new Promise(resolve => setTimeout(resolve, 200))
    return { ...this.robotInfo }
  }

  async updateRobotInfo(info: Partial<RobotInfo>): Promise<RobotInfo> {
    await new Promise(resolve => setTimeout(resolve, 200))
    this.robotInfo = { ...this.robotInfo, ...info, lastUpdated: new Date().toISOString() }
    return { ...this.robotInfo }
  }
}

/**
 * WebSocket 服务实现
 */
export class WebSocketPanelService implements PanelWsService {
  private ws: WebSocket | null = null
  private messageCallbacks: Array<(data: unknown) => void> = []

  connect(url: string): void {
    if (this.ws) {
      this.disconnect()
    }

    this.ws = new WebSocket(url)

    this.ws.onmessage = event => {
      try {
        const data = JSON.parse(event.data)
        this.messageCallbacks.forEach(cb => cb(data))
      } catch {
        console.warn('Failed to parse WebSocket message')
      }
    }

    this.ws.onerror = error => {
      console.error('WebSocket error:', error)
    }
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
    this.messageCallbacks = []
  }

  isConnected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN
  }

  onMessage(callback: (data: unknown) => void): void {
    this.messageCallbacks.push(callback)
  }

  send(data: unknown): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data))
    }
  }
}
