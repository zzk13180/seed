import { withInstall } from '@seed/element-plus-utils'

import Scrollbar from './src/scrollbar.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElScrollbar: SFCWithInstall<typeof Scrollbar> = withInstall(Scrollbar)
export default ElScrollbar

export * from './src/util'
export * from './src/scrollbar'
export * from './src/thumb'
export * from './src/constants'
