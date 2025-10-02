import { componentSizeMap } from '@seed/element-plus-constants'

import type { ComponentSize } from '@seed/element-plus-constants'

export const getComponentSize = (size?: ComponentSize) => {
  return componentSizeMap[size || 'default']
}
