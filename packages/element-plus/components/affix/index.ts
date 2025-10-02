import { withInstall } from '@seed/element-plus-utils'
import Affix from './src/affix.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElAffix: SFCWithInstall<typeof Affix> = withInstall(Affix)
export default ElAffix

export * from './src/affix'
