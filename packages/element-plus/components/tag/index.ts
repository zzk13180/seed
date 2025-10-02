import { withInstall } from '@seed/element-plus-utils'

import Tag from './src/tag.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElTag: SFCWithInstall<typeof Tag> = withInstall(Tag)
export default ElTag

export * from './src/tag'
