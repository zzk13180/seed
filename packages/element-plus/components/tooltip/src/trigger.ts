import { buildProps, definePropType } from '@seed/element-plus-utils'
import { popperTriggerProps } from '@seed/element-plus-components/popper'
import { EVENT_CODE } from '@seed/element-plus-constants'
import type { Arrayable } from '@seed/element-plus-utils'
import type { ExtractPropTypes } from 'vue'

export type TooltipTriggerType = 'hover' | 'focus' | 'click' | 'contextmenu'

export const useTooltipTriggerProps = buildProps({
  ...popperTriggerProps,
  /**
   * @description whether Tooltip is disabled
   */
  disabled: Boolean,
  /**
   * @description How should the tooltip be triggered (to show)
   */
  trigger: {
    type: definePropType<Arrayable<TooltipTriggerType>>([String, Array]),
    default: 'hover',
  },
  /**
   * @description When you click the mouse to focus on the trigger element, you can define a set of keyboard codes to control the display of tooltip through the keyboard
   */
  triggerKeys: {
    type: definePropType<string[]>(Array),
    default: () => [EVENT_CODE.enter, EVENT_CODE.numpadEnter, EVENT_CODE.space],
  },
} as const)

export type ElTooltipTriggerProps = ExtractPropTypes<typeof useTooltipTriggerProps>
