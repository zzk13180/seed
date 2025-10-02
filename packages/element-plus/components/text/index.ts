import { withInstall } from '@seed/element-plus-utils'

import Text from './src/text.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElText: SFCWithInstall<typeof Text> = withInstall(Text)
export default ElText

export * from './src/text'
