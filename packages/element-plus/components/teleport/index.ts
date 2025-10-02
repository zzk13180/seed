import { withInstall } from '@seed/element-plus-utils'
import Teleport from './src/teleport.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElTeleport: SFCWithInstall<typeof Teleport> = withInstall(Teleport)

export default ElTeleport

export * from './src/teleport'
