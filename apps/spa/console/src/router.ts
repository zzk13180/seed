import { createRouter, createWebHistory } from '@ionic/vue-router'
import type { RouteRecordRaw } from 'vue-router'

/**
 * 路由配置
 */
const routes: readonly RouteRecordRaw[] = [
  {
    path: '/tabs',
    component: () => import('@/layout/Tabs.vue'),
    children: [
      {
        path: 'home',
        name: 'home',
        component: () => import('@/views/Home.vue'),
        meta: {
          title: '首页',
        },
      },
      {
        path: 'map',
        name: 'map',
        component: () => import('@/views/Map.vue'),
        meta: {
          title: '地图',
        },
      },
      {
        path: 'settings',
        name: 'settings',
        component: () => import('@/views/Settings.vue'),
        meta: {
          title: '设置',
        },
      },
    ],
  },
  {
    path: '/',
    redirect: '/tabs/home',
  },
]

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes,
})

// 路由守卫：设置页面标题
router.afterEach(to => {
  if (to.meta?.title) {
    document.title = `${to.meta.title as string} - Seed Mobile`
  }
})

export default router
