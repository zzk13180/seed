import { withInstall } from '@seed/element-plus-utils'
import TreeV2 from './src/tree.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElTreeV2: SFCWithInstall<typeof TreeV2> = withInstall(TreeV2)
export default ElTreeV2
