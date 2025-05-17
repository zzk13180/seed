import { useStorage, type RemovableRef } from '@vueuse/core'

const TOKEN_KEY = 'Access token'

export class AccessTokenUtil {
  private static storage: RemovableRef<null | string> = useStorage<null | string>(TOKEN_KEY, null)

  private constructor() {}

  public static get token(): string | null {
    return AccessTokenUtil.storage.value
  }

  public static setToken(access_token: string): void {
    AccessTokenUtil.storage.value = access_token
  }

  public static removeToken(): void {
    AccessTokenUtil.storage.value = null
  }
}
