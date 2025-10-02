import { withInstall, withNoopInstall } from '@seed/element-plus-utils'

import Breadcrumb from './src/breadcrumb.vue'
import BreadcrumbItem from './src/breadcrumb-item.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElBreadcrumb: SFCWithInstall<typeof Breadcrumb> & {
  BreadcrumbItem: typeof BreadcrumbItem
} = withInstall(Breadcrumb, {
  BreadcrumbItem,
})
export const ElBreadcrumbItem: SFCWithInstall<typeof BreadcrumbItem> =
  withNoopInstall(BreadcrumbItem)
export default ElBreadcrumb

export * from './src/breadcrumb'
export * from './src/breadcrumb-item'
export * from './src/constants'
export type { BreadcrumbInstance, BreadcrumbItemInstance } from './src/instances'
