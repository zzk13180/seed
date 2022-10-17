import { router as globalRouter } from '@seed/common/main/router'
import { RoutePathEnum } from '@seed/common/enums/route.enum'
import type { RouteLocationRaw, Router } from 'vue-router'

function handleError(e: Error) {
  console.error(e)
}

export function useGo(router: Router = globalRouter) {
  const { push, replace } = router
  function go(opt: RouteLocationRaw = RoutePathEnum.BASE_HOME, isReplace = false) {
    isReplace ? replace(opt).catch(handleError) : push(opt).catch(handleError)
  }
  return go
}
