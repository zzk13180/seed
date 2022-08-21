import { TheHome } from './TheHome'
import type { RouteRecordRaw } from 'vue-router'

const RootRoute: RouteRecordRaw = {
  path: '/',
  redirect: '/TheHome',
}

const HomeRoute: RouteRecordRaw = {
  path: '/TheHome',
  name: 'TheHome',
  component: TheHome,
  meta: {
    title: 'TheHome',
    hideTab: true,
    isBasic: true,
    ignoreAuth: true,
    ignoreKeepAlive: true,
  },
}

export default [RootRoute, HomeRoute]
