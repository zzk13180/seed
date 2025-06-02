import type { State } from './panel.store'

export class PanelController {
  private state: State

  constructor(state: State) {
    this.state = state
  }

  initialize() {
    console.log('PanelController initialized')
  }

  destroy() {
    console.log('PanelController destroyed')
  }

  get getRobotId(): string {
    return this.state.robotInfo.robotId || ''
  }
}
