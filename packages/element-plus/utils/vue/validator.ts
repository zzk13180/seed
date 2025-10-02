import { componentSizes, datePickTypes } from '@seed/element-plus-constants'
import type { ComponentSize, DatePickType } from '@seed/element-plus-constants'

export const isValidComponentSize = (val: string): val is ComponentSize | '' =>
  ['', ...componentSizes].includes(val)

export const isValidDatePickType = (val: string): val is DatePickType =>
  ([...datePickTypes] as string[]).includes(val)
