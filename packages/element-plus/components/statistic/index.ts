import { withInstall } from '@seed/element-plus-utils'
import Statistic from './src/statistic.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElStatistic: SFCWithInstall<typeof Statistic> = withInstall(Statistic)

export default ElStatistic
export * from './src/statistic'
