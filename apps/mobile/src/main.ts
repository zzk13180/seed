import { createApp } from 'vue'

// 应用样式
import '@/styles/index.scss'
import 'virtual:svg-icons-register'

// Ionic 核心样式
import '@ionic/vue/css/core.css'
import '@ionic/vue/css/normalize.css'
import '@ionic/vue/css/structure.css'
import '@ionic/vue/css/typography.css'

// Ionic 可选样式
import '@ionic/vue/css/padding.css'
import '@ionic/vue/css/float-elements.css'
import '@ionic/vue/css/text-alignment.css'
import '@ionic/vue/css/text-transformation.css'
import '@ionic/vue/css/flex-utils.css'
import '@ionic/vue/css/display.css'

// 主题变量
import './styles/theme/variables.css'
import './styles/theme/custom.css'
import '@ionic/core/css/core.css'
import '@ionic/core/css/ionic.bundle.css'

// 暗色模式
import '@ionic/vue/css/palettes/dark.class.css'

import { IonicVue } from '@ionic/vue'
import { register } from 'swiper/element/bundle'
import { createPinia } from 'pinia'

import App from './App.vue'
import SvgIcon from './components/SvgIcon.vue'
import router from './router'

// 注册 Swiper 自定义元素
register()

/**
 * 创建并挂载 Vue 应用
 */
async function bootstrap(): Promise<void> {
  const app = createApp(App)
  const store = createPinia()

  // 注册插件
  app.use(store)
  app.use(IonicVue, {
    mode: 'md', // Material Design 风格
  })
  app.use(router)

  // 注册全局组件
  app.component('SvgIcon', SvgIcon)

  // 等待路由就绪后挂载
  await router.isReady()
  app.mount('#app')
}

// 启动应用
try {
  await bootstrap()
} catch (error) {
  console.error('Failed to start app:', error)
}
