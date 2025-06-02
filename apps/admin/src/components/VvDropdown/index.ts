/* eslint-disable @typescript-eslint/no-redundant-type-constituents */

import { withInstall, withNoopInstall } from '@element-plus/utils'

import Dropdown from './src/VvDropdown.vue'
import DropdownItem from './src/VvDropdownItem.vue'
import DropdownMenu from './src/VvDropdownMenu.vue'
import type { SFCWithInstall } from '@element-plus/utils'

export const TheDropdown: SFCWithInstall<typeof Dropdown> & {
  DropdownItem: typeof DropdownItem
  DropdownMenu: typeof DropdownMenu
} = withInstall(Dropdown, {
  DropdownItem,
  DropdownMenu,
})
export default TheDropdown
export const TheDropdownItem: SFCWithInstall<typeof DropdownItem> = withNoopInstall(DropdownItem)
export const TheDropdownMenu: SFCWithInstall<typeof DropdownMenu> = withNoopInstall(DropdownMenu)
export * from './src/dropdown'
export * from './src/instance'
export * from './src/tokens'
