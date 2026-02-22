import type { Subscription } from 'rxjs'

export interface AppState {
  rosBridgeConnected: boolean
  colorTypes: string[]
  toastMessage: string
}

export interface AppDeps {}

export type { Subscription }
