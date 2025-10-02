import { withInstall } from '@seed/element-plus-utils'
import InputNumber from './src/input-number.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElInputNumber: SFCWithInstall<typeof InputNumber> = withInstall(InputNumber)

export default ElInputNumber
export * from './src/input-number'
