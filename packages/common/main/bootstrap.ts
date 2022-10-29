import 'virtual:windi.css'
import 'virtual:windi-devtools'
import '../../../theme/index.less'
import { createApp, h, resolveComponent } from 'vue'
import { idux, IduxProvider } from '@seed/ct/idux'
import { router } from './router'
import { store } from './store'

import type { RouteRecordRaw } from 'vue-router'

export async function bootstrap(routes: RouteRecordRaw[], id?: string): Promise<any> {
  const render = () => {
    return h(
      IduxProvider,
      {},
      {
        default: () => h(resolveComponent('router-view')),
      },
    )
  }
  const app = createApp({ render })
  app.use(idux)
  routes.forEach(route => router.addRoute(route))
  app.use(router)
  app.use(store)
  await router.isReady()
  app.mount(id || '#app')
  return app
}
