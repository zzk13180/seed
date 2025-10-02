import { withInstall } from '@seed/element-plus-utils'
import Drawer from './src/drawer.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElDrawer: SFCWithInstall<typeof Drawer> = withInstall(Drawer)
export default ElDrawer

export * from './src/drawer'
