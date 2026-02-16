import { BaseController } from '@seed/kit/frontend'
import type { Subscription } from 'rxjs'
import type { State } from './app.store'

interface AppDeps {}

export class AppController extends BaseController<State, AppDeps> {
  readonly subscriptions: Subscription[] = []

  constructor(state: State) {
    super(state, {})
  }

  protected onInit(): Promise<void> {
    console.log('AppController: 初始化')
    return Promise.resolve()
  }

  protected onDispose(): Promise<void> {
    this.subscriptions.forEach(s => s.unsubscribe())
    console.log('AppController: 销毁')
    return Promise.resolve()
  }
}
