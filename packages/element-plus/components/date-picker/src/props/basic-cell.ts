import { buildProps, definePropType } from '@seed/element-plus-utils'

import type { ExtractPropTypes } from 'vue'
import type { DateCell } from '../date-picker.type'

export const basicCellProps = buildProps({
  cell: {
    type: definePropType<DateCell>(Object),
  },
} as const)

export type BasicCellProps = ExtractPropTypes<typeof basicCellProps>
