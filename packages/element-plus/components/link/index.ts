import { withInstall } from '@seed/element-plus-utils'

import Link from './src/link.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElLink: SFCWithInstall<typeof Link> = withInstall(Link)
export default ElLink

export * from './src/link'
