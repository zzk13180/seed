import { buildProps, definePropType } from '@seed/element-plus-utils'
import { timePanelSharedProps } from './shared'

import type { ExtractPropTypes } from 'vue'
import type { Dayjs } from 'dayjs'

export const panelTimeRangeProps = buildProps({
  ...timePanelSharedProps,
  parsedValue: {
    type: definePropType<[Dayjs, Dayjs]>(Array),
  },
} as const)

export type PanelTimeRangeProps = ExtractPropTypes<typeof panelTimeRangeProps>
