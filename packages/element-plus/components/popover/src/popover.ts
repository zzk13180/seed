import { buildProps, definePropType, isBoolean } from '@element-plus/utils'
import { EVENT_CODE } from '@element-plus/constants'

import { useTooltipContentProps, useTooltipTriggerProps } from '@element-plus/components/tooltip'
import type { ButtonProps, ButtonType } from '@element-plus/components/button'
import type { Placement } from '@element-plus/components/popper'
import type { ExtractPropTypes, PropType } from 'vue'
import type Popover from './popover.vue'
import type { Options } from '@popperjs/core'

export const dropdownProps = buildProps({
  /**
   * @description how to trigger
   */
  trigger: useTooltipTriggerProps.trigger,
  triggerKeys: {
    type: definePropType<string[]>(Array),
    default: () => [EVENT_CODE.enter, EVENT_CODE.numpadEnter, EVENT_CODE.space, EVENT_CODE.down],
  },
  effect: {
    ...useTooltipContentProps.effect,
    default: 'light',
  },
  /**
   * @description menu button type, refer to `Button` Component, only works when `split-button` is true
   */
  type: {
    type: definePropType<ButtonType>(String),
  },
  /**
   * @description placement of pop menu
   */
  placement: {
    type: definePropType<Placement>(String),
    default: 'bottom',
  },
  /**
   * @description [popper.js](https://popper.js.org/docs/v2/) parameters
   */
  popperOptions: {
    type: definePropType<Partial<Options>>(Object),
    default: () => ({}),
  },
  id: String,
  /**
   * @description menu size, also works on the split button
   */
  size: {
    type: String,
    default: '',
  },
  /**
   * @description whether a button group is displayed
   */
  splitButton: Boolean,
  /**
   * @description whether to hide menu after clicking menu-item
   */
  hideOnClick: {
    type: Boolean,
    default: true,
  },
  loop: {
    type: Boolean,
    default: true,
  },
  /**
   * @description delay time before show a dropdown (only works when trigger is `hover`)
   */
  showTimeout: {
    type: Number,
    default: 150,
  },
  /**
   * @description delay time before hide a dropdown (only works when trigger is `hover`)
   */
  hideTimeout: {
    type: Number,
    default: 150,
  },
  /**
   * @description [tabindex](https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes/tabindex) of Dropdown
   */
  tabindex: {
    type: definePropType<number | string>([Number, String]),
    default: 0,
  },
  /**
   * @description the max height of menu
   */
  maxHeight: {
    type: definePropType<number | string>([Number, String]),
    default: '',
  },
  /**
   * @description custom class name for Dropdown's dropdown
   */
  popperClass: {
    type: String,
    default: '',
  },
  /**
   * @description whether to disable
   */
  disabled: Boolean,
  /**
   * @description the ARIA role attribute for the dropdown menu. Depending on the use case, you may want to change this to 'navigation'
   */
  role: {
    type: String,
    default: 'menu',
  },
  buttonProps: {
    type: definePropType<ButtonProps>(Object),
  },
  /**
   * @description whether the dropdown popup is teleported to the body
   */
  teleported: useTooltipContentProps.teleported,
} as const)

export const popoverProps = buildProps({
  /**
   * @description how the popover is triggered
   */
  trigger: useTooltipTriggerProps.trigger,
  /**
   * @description popover placement
   */
  placement: dropdownProps.placement,
  /**
   * @description whether Popover is disabled
   */
  disabled: useTooltipTriggerProps.disabled,
  /**
   * @description whether popover is visible
   */
  visible: useTooltipContentProps.visible,
  /**
   * @description popover transition animation
   */
  transition: useTooltipContentProps.transition,
  /**
   * @description parameters for [popper.js](https://popper.js.org/docs/v2/)
   */
  popperOptions: dropdownProps.popperOptions,
  /**
   * @description [tabindex](https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes/tabindex) of Popover
   */
  tabindex: dropdownProps.tabindex,
  /**
   * @description popover content, can be replaced with a default `slot`
   */
  content: useTooltipContentProps.content,
  /**
   * @description custom style for popover
   */
  popperStyle: useTooltipContentProps.popperStyle,
  /**
   * @description custom class name for popover
   */
  popperClass: useTooltipContentProps.popperClass,
  enterable: {
    ...useTooltipContentProps.enterable,
    default: true,
  },
  /**
   * @description Tooltip theme, built-in theme: `dark` / `light`
   */
  effect: {
    ...useTooltipContentProps.effect,
    default: 'light',
  },
  /**
   * @description whether popover dropdown is teleported to the body
   */
  teleported: useTooltipContentProps.teleported,
  /**
   * @description popover title
   */
  title: String,
  /**
   * @description popover width
   */
  width: {
    type: [String, Number],
    default: 150,
  },
  /**
   * @description popover offset
   */
  offset: {
    type: Number,
    default: undefined,
  },
  /**
   * @description delay of appearance, in millisecond
   */
  showAfter: {
    type: Number,
    default: 0,
  },
  /**
   * @description delay of disappear, in millisecond
   */
  hideAfter: {
    type: Number,
    default: 200,
  },
  /**
   * @description timeout in milliseconds to hide tooltip
   */
  autoClose: {
    type: Number,
    default: 0,
  },
  /**
   * @description whether a tooltip arrow is displayed or not. For more info, please refer to [ElPopper](https://github.com/element-plus/element-plus/tree/dev/packages/components/popper)
   */
  showArrow: {
    type: Boolean,
    default: true,
  },
  /**
   * @description when popover inactive and `persistent` is `false` , popover will be destroyed
   */
  persistent: {
    type: Boolean,
    default: true,
  },
  'onUpdate:visible': {
    type: Function as PropType<(visible: boolean) => void>,
  },
} as const)
export type PopoverProps = ExtractPropTypes<typeof popoverProps>

export const popoverEmits = {
  'update:visible': (value: boolean) => isBoolean(value),
  'before-enter': () => true,
  'before-leave': () => true,
  'after-enter': () => true,
  'after-leave': () => true,
}
export type PopoverEmits = typeof popoverEmits

export type PopoverInstance = InstanceType<typeof Popover>
