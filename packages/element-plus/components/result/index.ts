import { withInstall } from '@seed/element-plus-utils'
import Result from './src/result.vue'
import type { SFCWithInstall } from '@seed/element-plus-utils'

export const ElResult: SFCWithInstall<typeof Result> = withInstall(Result)

export default ElResult

export * from './src/result'
