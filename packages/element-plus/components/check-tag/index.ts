import { withInstall } from '@seed/element-plus-utils'
import CheckTag from './src/check-tag.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElCheckTag: SFCWithInstall<typeof CheckTag> = withInstall(CheckTag)
export default ElCheckTag

export * from './src/check-tag'
