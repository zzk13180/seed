import { distinctUntilChanged } from 'rxjs'
import type { Subscription } from 'rxjs'
import type { State } from './app.store'

export class AppController {
  private readonly state: State

  private readonly subscriptions: Subscription[] = []

  constructor(state: State) {
    this.state = state
  }

  initialize() {
    console.log('AppController: 初始化')
  }

  destroy() {
    console.log('AppController: 销毁')
  }
}
