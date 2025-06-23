import { useStorage, type RemovableRef } from '@vueuse/core'

const TOKEN_KEY = 'Access token'

export class AccessTokenUtil {
  static get token(): string | null {
    return AccessTokenUtil.storage.value
  }

  static setToken(access_token: string): void {
    AccessTokenUtil.storage.value = access_token
  }

  static removeToken(): void {
    AccessTokenUtil.storage.value = null
  }

  private static readonly storage: RemovableRef<null | string> = useStorage<null | string>(
    TOKEN_KEY,
    null,
  )

  private constructor() {}
}
