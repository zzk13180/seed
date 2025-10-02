import { withInstall } from '@seed/element-plus-utils'
import TreeSelect from './src/tree-select.vue'

import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElTreeSelect: SFCWithInstall<typeof TreeSelect> = withInstall(TreeSelect)

export default ElTreeSelect
