import { withInstall } from '@seed/element-plus-utils'
import Badge from './src/badge.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElBadge: SFCWithInstall<typeof Badge> = withInstall(Badge)
export default ElBadge

export * from './src/badge'
export type { BadgeInstance } from './src/instance'
