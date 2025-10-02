import { withInstall } from '@seed/element-plus-utils'
import Cascader from './src/cascader.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElCascader: SFCWithInstall<typeof Cascader> = withInstall(Cascader)

export default ElCascader

export * from './src/cascader'
export * from './src/instances'
