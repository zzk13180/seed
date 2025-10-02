import { withInstall } from '@seed/element-plus-utils'
import Countdown from './src/countdown.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElCountdown: SFCWithInstall<typeof Countdown> = withInstall(Countdown)
export default ElCountdown

export * from './src/countdown'
