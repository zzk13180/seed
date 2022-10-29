import { createRouter, createWebHashHistory } from 'vue-router'

export const router = createRouter({
  history: createWebHashHistory('/'),
  routes: [],
  strict: true,
  scrollBehavior: () => ({ left: 0, top: 0 }),
})

export function resetRouter() {
  router.getRoutes().forEach(route => {
    const { name, meta } = route
    if (name && meta && !meta.isBasic) {
      if (router.hasRoute(name)) {
        router.removeRoute(name)
      }
    }
  })
}
