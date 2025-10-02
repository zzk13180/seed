import { buildProps, definePropType } from '@seed/element-plus-utils'

import type { DescriptionItemVNode } from './description-item'

export const descriptionsRowProps = buildProps({
  row: {
    type: definePropType<DescriptionItemVNode[]>(Array),
    default: () => [],
  },
} as const)
