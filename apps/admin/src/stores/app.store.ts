import { reactive } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'

export interface State {
  title: string
  sidebarIsHidden: boolean
}

export const useAppStore = defineStore('app', () => {
  const state: State = reactive({
    title: 'Title',
    sidebarIsHidden: false,
  })

  const setSidebarIsHidden = (isHidden: boolean) => {
    state.sidebarIsHidden = isHidden
  }

  const controller = {
    setSidebarIsHidden,
  }

  return { state, controller }
})

if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useAppStore, import.meta.hot))
}
