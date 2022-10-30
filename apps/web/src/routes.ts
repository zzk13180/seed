import { TheHome } from './TheHome'
import { TheDemo } from './TheDemo'
import type { RouteRecordRaw } from 'vue-router'

export const RootRoute: RouteRecordRaw = {
  path: '/',
  redirect: '/TheHome',
}

export const HomeRoute: RouteRecordRaw = {
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

export const HexBoardRoute: RouteRecordRaw = {
  path: '/HexBoard',
  name: 'HexBoard',
  component: TheDemo,
  meta: {
    title: 'HexBoard',
    hideTab: true,
    isBasic: true,
    ignoreAuth: true,
    ignoreKeepAlive: true,
  },
}
