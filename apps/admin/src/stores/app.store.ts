import { reactive } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'

/**
 * 应用状态接口
 */
export interface AppState {
  /** 页面标题 */
  title: string
  /** 侧边栏是否隐藏 */
  sidebarIsHidden: boolean
  /** 侧边栏是否折叠 */
  sidebarCollapsed: boolean
  /** 当前主题 */
  theme: 'light' | 'dark' | 'system'
}

/**
 * 应用全局状态管理
 */
export const useAppStore = defineStore('app', () => {
  // 状态
  const state = reactive<AppState>({
    title: 'Seed Admin',
    sidebarIsHidden: false,
    sidebarCollapsed: false,
    theme: 'light',
  })

  // Actions
  /**
   * 设置侧边栏显示/隐藏
   */
  function setSidebarIsHidden(isHidden: boolean): void {
    state.sidebarIsHidden = isHidden
  }

  /**
   * 切换侧边栏显示/隐藏
   */
  function toggleSidebar(): void {
    state.sidebarIsHidden = !state.sidebarIsHidden
  }

  /**
   * 设置侧边栏折叠状态
   */
  function setSidebarCollapsed(collapsed: boolean): void {
    state.sidebarCollapsed = collapsed
  }

  /**
   * 切换侧边栏折叠状态
   */
  function toggleSidebarCollapsed(): void {
    state.sidebarCollapsed = !state.sidebarCollapsed
  }

  /**
   * 设置页面标题
   */
  function setTitle(title: string): void {
    state.title = title
    document.title = `${title} - Seed Admin`
  }

  /**
   * 设置主题
   */
  function setTheme(theme: AppState['theme']): void {
    state.theme = theme
    // 应用主题到 HTML 元素
    if (theme === 'dark') {
      document.documentElement.classList.add('dark')
    } else {
      document.documentElement.classList.remove('dark')
    }
  }

  return {
    state,
    // Actions
    setSidebarIsHidden,
    toggleSidebar,
    setSidebarCollapsed,
    toggleSidebarCollapsed,
    setTitle,
    setTheme,
  }
})

// 热模块替换支持
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(useAppStore, import.meta.hot))
}
