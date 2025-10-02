import { createApp } from 'vue'

import '@/styles/index.scss'
import 'virtual:svg-icons-register'

/* Core CSS required for Ionic components to work properly */
import '@ionic/vue/css/core.css'

/* Basic CSS for apps built with Ionic */
import '@ionic/vue/css/normalize.css'
import '@ionic/vue/css/structure.css'
import '@ionic/vue/css/typography.css'

/* Optional CSS utils that can be commented out */
import '@ionic/vue/css/padding.css'
import '@ionic/vue/css/float-elements.css'
import '@ionic/vue/css/text-alignment.css'
import '@ionic/vue/css/text-transformation.css'
import '@ionic/vue/css/flex-utils.css'
import '@ionic/vue/css/display.css'

/* Theme variables */
import './styles/theme/variables.css'
import './styles/theme/custom.css'
import '@ionic/core/css/core.css'
import '@ionic/core/css/ionic.bundle.css'

/**
 * Ionic Dark Mode
 * -----------------------------------------------------
 * For more info, please see:
 * https://ionicframework.com/docs/theming/dark-mode
 */

// import "@ionic/vue/css/palettes/dark.always.css";
// import "@ionic/vue/css/palettes/dark.system.css";
import '@ionic/vue/css/palettes/dark.class.css'

import { IonicVue } from '@ionic/vue'

import { register } from 'swiper/element/bundle'

import { createPinia } from 'pinia'
import App from './App.vue'
import SvgIcon from './components/SvgIcon.vue'
import router from './router'

register()

const app = createApp(App)

const store = createPinia()
app.use(store)
app.use(IonicVue, {
  mode: 'md',
})
app.use(router)

app.component('SvgIcon', SvgIcon)

// eslint-disable-next-line @typescript-eslint/no-floating-promises, unicorn/prefer-top-level-await
router.isReady().then(() => {
  app.mount('#app')
})
