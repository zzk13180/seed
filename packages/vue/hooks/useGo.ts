import { router as globalRouter } from '../main/router'
import { RoutePathEnum } from '../enums/route.enum'
import type { RouteLocationRaw, Router } from 'vue-router'

function handleError(e: Error) {
  console.error(e)
}

export function useGo(router: Router = globalRouter) {
  const { push, replace } = router
  function go(opt: RouteLocationRaw = RoutePathEnum.BASE_HOME, isReplace = false) {
    if (isReplace) {
      replace(opt).catch(handleError)
    } else {
      push(opt).catch(handleError)
    }
  }
  return go
}
