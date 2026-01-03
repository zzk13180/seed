import type { Subscription } from 'rxjs'
import type { State } from './app.store'

export class AppController {
  readonly state: State

  readonly subscriptions: Subscription[] = []

  constructor(state: State) {
    this.state = state
  }

  initialize() {
    console.log('AppController: 初始化')
  }

  destroy() {
    this.subscriptions.forEach(s => s.unsubscribe())
    console.log('AppController: 销毁')
  }
}
