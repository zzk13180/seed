import { withInstall, withInstallDirective } from '@seed/element-plus-utils'

import Popover from './src/popover.vue'
import PopoverDirective, { VPopover } from './src/directive'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElPopoverDirective: SFCWithInstall<typeof PopoverDirective> = withInstallDirective(
  PopoverDirective,
  VPopover,
)

export const ElPopover: SFCWithInstall<typeof Popover> & {
  directive: typeof ElPopoverDirective
} = withInstall(Popover, {
  directive: ElPopoverDirective,
})
export default ElPopover

export * from './src/popover'
