import { timePickerDefaultProps } from '@seed/element-plus-components/time-picker'
import { buildProps, definePropType } from '@seed/element-plus-utils'

import type { ExtractPropTypes } from 'vue'
import type { IDatePickerType } from '../date-picker.type'

export const datePickerProps = buildProps({
  ...timePickerDefaultProps,
  /**
   * @description type of the picker
   */
  type: {
    type: definePropType<IDatePickerType>(String),
    default: 'date',
  },
} as const)

export type DatePickerProps = ExtractPropTypes<typeof datePickerProps>
