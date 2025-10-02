import { withInstall } from '@seed/element-plus-utils'
import InputTag from './src/input-tag.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElInputTag: SFCWithInstall<typeof InputTag> = withInstall(InputTag)
export default ElInputTag

export * from './src/input-tag'
export type { InputTagInstance } from './src/instance'
