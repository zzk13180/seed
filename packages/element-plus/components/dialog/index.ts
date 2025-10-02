import { withInstall } from '@seed/element-plus-utils'
import Dialog from './src/dialog.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElDialog: SFCWithInstall<typeof Dialog> = withInstall(Dialog)
export default ElDialog

export * from './src/use-dialog'
export * from './src/dialog'
export * from './src/constants'
