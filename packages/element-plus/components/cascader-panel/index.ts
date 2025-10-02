import { withInstall } from '@seed/element-plus-utils'
import CascaderPanel from './src/index.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElCascaderPanel: SFCWithInstall<typeof CascaderPanel> = withInstall(CascaderPanel)

export default ElCascaderPanel
export * from './src/types'
export * from './src/config'
export * from './src/instance'
