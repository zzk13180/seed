import { TheHome } from './TheHome'
import { HexBoard } from './HexBoard'
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

const HexBoardRoute: RouteRecordRaw = {
  path: '/HexBoard',
  name: 'HexBoard',
  component: HexBoard,
  meta: {
    title: 'HexBoard',
    hideTab: true,
    isBasic: true,
    ignoreAuth: true,
    ignoreKeepAlive: true,
  },
}

export default [RootRoute, HomeRoute, HexBoardRoute]
