import { reactive } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { AppController } from './app.controller'

export interface State {
  rosBridgeConnected: boolean
  colorTypes: string[]
  toastMessage: string
}

export const useAppStore = defineStore('app', () => {
  const state: State = reactive({
    toastMessage: '',
    rosBridgeConnected: false,
    colorTypes: [
      'ionic',
      'vue',
      'communication',
      'tooling',
      'services',
      'design',
      'workshop',
      'food',
      'documentation',
      'navigation',
    ],
  })

  const controller = new AppController(state)

  return { state, controller }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useAppStore, import.meta.hot))
}
