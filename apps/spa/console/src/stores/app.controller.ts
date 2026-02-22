import { BaseController } from '@seed/kit/frontend'
import type { Subscription, AppState, AppDeps } from './app.types'

export class AppController extends BaseController<AppState, AppDeps> {
  readonly subscriptions: Subscription[] = []

  constructor(state: AppState) {
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
