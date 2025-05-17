import '@/assets/styles/tailwindcss.css'
import '@/assets/styles/index.scss'
import 'virtual:svg-icons-register'
import { createApp, type Component } from 'vue'
import { createPinia } from 'pinia'

import App from './App.vue'
import { router } from './pages/router'

import SvgIcon from '@/components/SvgIcon.vue'

const app = createApp(App as Component)
const store = createPinia()

app.use(store)
app.use(router)
app.component('SvgIcon', SvgIcon)
app.mount('#app')

if (import.meta.env.MODE === 'development') {
  console.log('env:', import.meta.env)
}

document.addEventListener('DOMContentLoaded', () => {
  document.documentElement.classList.add('theme-default')
  const link = document.querySelector("link[rel*='icon']")
  if (link instanceof HTMLLinkElement) {
    link.type = 'image/svg+xml'
    link.href = '/favicon/default.svg'
    document.getElementsByTagName('head')[0].appendChild(link)
  }
})
