import { createRouter, createWebHistory } from 'vue-router'
import { AccessTokenUtil } from '@/utils/token.util'
import TheLayout from '@/layout/TheLayout.vue'
import type { RouteRecordRaw } from 'vue-router'

const routes: Array<RouteRecordRaw> = [
  {
    path: '/',
    component: TheLayout,
    redirect: '/map',
    meta: {
      hideMenu: true,
    },
    children: [
      {
        path: '/map',
        component: () => import('./map.vue'),
        meta: {
          requiresAuth: true,
          title: '地图导航',
          icon: 'map',
        },
      },
    ],
  },
  {
    path: '/panel',
    component: TheLayout,
    meta: {
      hideMenu: true,
    },
    children: [
      {
        path: '/panel',
        component: () => import('./panel.vue'),
        meta: {
          requiresAuth: true,
          title: '控制面板',
          icon: 'panel-control',
        },
      },
    ],
  },
  {
    path: '/user',
    component: TheLayout,
    meta: {
      hideMenu: true,
    },
    children: [
      {
        path: 'profile',
        component: () => import('./user-profile.vue'),
        meta: {
          requiresAuth: true,
          hideMenu: true,
        },
      },
    ],
  },
  {
    path: '/login',
    component: () => import('./login.vue'),
    meta: {
      hideMenu: true,
    },
  },
  {
    path: '/redirect',
    component: () => import('./redirect.vue'),
    meta: {
      hideMenu: true,
    },
  },
  {
    path: '/:pathMatch(.*)*',
    component: () => import('./404.vue'),
    meta: {
      hideMenu: true,
    },
  },
  {
    path: '/401',
    component: () => import('./401.vue'),
    meta: {
      hideMenu: true,
    },
  },
]

export const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_PATH),
  routes,
  scrollBehavior(to, from, savedPosition) {
    if (savedPosition) {
      return savedPosition
    } else {
      return { top: 0 }
    }
  },
  strict: true,
})

router.beforeEach((to, from, next) => {
  if (to.meta.requiresAuth && !AccessTokenUtil.token) {
    next({
      path: '/login',
      query: { redirect: to.fullPath },
    })
  } else {
    next()
  }
})
