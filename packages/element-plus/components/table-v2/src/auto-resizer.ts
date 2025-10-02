import { buildProps, definePropType } from '@seed/element-plus-utils'

import type { ExtractPropTypes } from 'vue'

type AutoResizeHandler = (event: { height: number; width: number }) => void

export const autoResizerProps = buildProps({
  disableWidth: Boolean,
  disableHeight: Boolean,
  onResize: {
    type: definePropType<AutoResizeHandler>(Function),
  },
} as const)

export type AutoResizerProps = ExtractPropTypes<typeof autoResizerProps>
