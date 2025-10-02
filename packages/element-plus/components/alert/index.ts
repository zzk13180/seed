import { withInstall } from '@seed/element-plus-utils'
import Alert from './src/alert.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElAlert: SFCWithInstall<typeof Alert> = withInstall(Alert)
export default ElAlert

export * from './src/alert'
export type { AlertInstance } from './src/instance'
