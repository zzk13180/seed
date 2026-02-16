import { createRouter, createWebHistory } from 'vue-router'
import TheLayout from '@/layout/TheLayout.vue'
import type { RouteRecordRaw, NavigationGuardNext, RouteLocationNormalized } from 'vue-router'

/**
 * 白名单路由（无需认证）
 */
const WHITE_LIST = new Set(['/login', '/redirect', '/401', '/404'])

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
          title: 'routes',
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
          title: 'routes',
          icon: 'map',
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
    path: '/system',
    component: TheLayout,
    meta: {
      title: '系统管理',
      icon: 'setting',
    },
    children: [
      {
        path: 'users',
        component: () => import('./users.vue'),
        meta: {
          requiresAuth: true,
          title: '用户管理',
          icon: 'user',
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
  scrollBehavior(_to, _from, savedPosition) {
    if (savedPosition) {
      return savedPosition
    }
    return { top: 0 }
  },
  strict: true,
})

/**
 * 路由前置守卫
 *
 * Better Auth 使用 cookie-session 模式:
 * - 通过 user store 的 isLoggedIn 判断认证状态
 * - session cookie 由浏览器自动管理，无需手动检查 token
 */
router.beforeEach(
  async (
    to: RouteLocationNormalized,
    _from: RouteLocationNormalized,
    next: NavigationGuardNext,
  ) => {
    // 设置页面标题
    if (to.meta?.title) {
      document.title = `${to.meta.title as string} - Seed Admin`
    }

    // 白名单路由直接放行
    if (WHITE_LIST.has(to.path)) {
      next()
      return
    }

    // 检查认证状态
    const requiresAuth = to.matched.some(record => record.meta.requiresAuth)

    if (requiresAuth) {
      // 延迟导入避免循环依赖
      const { useUserStore } = await import('@/stores/user/user.store')
      const userStore = useUserStore()

      // 如果 store 中没有用户信息，尝试从 session 获取
      if (!userStore.isLoggedIn) {
        await userStore.controller.init()
      }

      // 仍然未登录，跳转到登录页
      if (!userStore.isLoggedIn) {
        next({
          path: '/login',
          query: { redirect: to.fullPath },
        })
        return
      }

      // Admin 后台要求管理员角色
      if (!userStore.isAdmin) {
        await userStore.controller.logout()
        next({ path: '/401' })
        return
      }
    }

    next()
  },
)
