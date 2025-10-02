import { withInstall } from '@seed/element-plus-utils'

import Image from './src/image.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElImage: SFCWithInstall<typeof Image> = withInstall(Image)
export default ElImage

export * from './src/image'
