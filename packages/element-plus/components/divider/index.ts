import { withInstall } from '@seed/element-plus-utils'

import Divider from './src/divider.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElDivider: SFCWithInstall<typeof Divider> = withInstall(Divider)
export default ElDivider

export * from './src/divider'
