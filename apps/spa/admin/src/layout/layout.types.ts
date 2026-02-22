import type { RouteRecordRaw } from 'vue-router'

export interface SidebarRouteItem extends Omit<RouteRecordRaw, 'children' | 'meta'> {
  hidden?: boolean
  alwaysShow?: boolean
  meta?: {
    title?: string
    icon?: string
    activeMenu?: string
    [key: string]: any
  }
  children?: SidebarRouteItem[]
  noShowingChildren?: boolean
  query?: string
  noVisibleChildren?: boolean
}
