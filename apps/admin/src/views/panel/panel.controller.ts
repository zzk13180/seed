import type { State, LogEntry } from './panel.store'
// import { apiGetRobotInfo } from '@/api/robot-info.api'

export class PanelController {
  private state: State

  constructor(state: State) {
    this.state = state
  }

  addLog(message: string, type: LogEntry['type'] = 'info') {
    const now = new Date()
    const timeStr = now.toLocaleTimeString()
    this.state.logs.unshift({
      time: timeStr,
      message,
      type,
    })
    if (this.state.logs.length > 50) {
      this.state.logs = this.state.logs.slice(0, 50)
    }
  }

  clearLogs() {
    this.state.logs = []
  }

  onComponentMounted() {
    this.addLog('机器人控制面板已初始化，请连接到机器人')
  }

  get getRobotId(): string {
    return this.state.robotInfo.robotId || ''
  }
}
