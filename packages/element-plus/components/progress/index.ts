import { withInstall } from '@seed/element-plus-utils'
import Progress from './src/progress.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElProgress: SFCWithInstall<typeof Progress> = withInstall(Progress)
export default ElProgress

export * from './src/progress'
