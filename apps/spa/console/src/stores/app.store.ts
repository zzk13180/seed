import { reactive, markRaw } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { AppController } from './app.controller'
import type { AppState } from './app.types'

export const useAppStore = defineStore('app', () => {
  const state: AppState = reactive({
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

  const controller = markRaw(new AppController(state))

  return { state, controller }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useAppStore, import.meta.hot))
}
