import { withInstall } from '@seed/element-plus-utils'

import Transfer from './src/transfer.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElTransfer: SFCWithInstall<typeof Transfer> = withInstall(Transfer)
export default ElTransfer

export * from './src/transfer'
