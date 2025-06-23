import type { State } from './panel.store'

export class PanelController {
  private readonly state: State

  constructor(state: State) {
    this.state = state
  }

  get getRobotId(): string {
    return this.state.robotInfo.robotId || ''
  }

  initialize() {
    console.log('PanelController initialized')
  }

  destroy() {
    console.log('PanelController destroyed')
  }
}
