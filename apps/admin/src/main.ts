import '@/assets/styles/tailwindcss.css'
import '@/assets/styles/index.scss'
import 'virtual:svg-icons-register'
import { createApp, type Component } from 'vue'
import { createPinia } from 'pinia'

import SvgIcon from '@/components/SvgIcon.vue'
import App from './App.vue'
import { router } from './pages/router'

/**
 * 初始化应用主题
 */
function initializeTheme(): void {
  document.documentElement.classList.add('theme-default')

  const link = document.querySelector("link[rel*='icon']")
  if (link instanceof HTMLLinkElement) {
    link.type = 'image/svg+xml'
    link.href = '/favicon/default.svg'
    document.head.append(link)
  }
}

/**
 * 创建并挂载 Vue 应用
 */
function bootstrap(): void {
  const app = createApp(App as Component)
  const store = createPinia()

  // 注册插件
  app.use(store)
  app.use(router)

  // 注册全局组件
  app.component('SvgIcon', SvgIcon)

  // 挂载应用
  app.mount('#app')

  // 开发环境日志
  if (import.meta.env.DEV) {
    console.log('🚀 App started in development mode')
    console.log('📋 Environment:', import.meta.env)
  }
}

// 初始化主题
document.addEventListener('DOMContentLoaded', initializeTheme)

// 启动应用
bootstrap()
