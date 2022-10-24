import axios from 'axios'
import { isFunction } from '@seed/common/utils/is.util'
import type { AxiosRequestConfig, Canceler } from 'axios'

let pendingMap = new Map<symbol, Canceler>()
const CancelToken = axios.CancelToken

export class VAxiosCanceler {
  addPending(config: AxiosRequestConfig) {
    const getPendingKey = (config: AxiosRequestConfig) =>
      Symbol([config.method, config.url].join('&'))
    const pendingKey = getPendingKey(config)
    config.pendingKey = pendingKey
    config.cancelToken = new CancelToken((cancel) => pendingMap.set(pendingKey, cancel))
  }

  removePending(config: AxiosRequestConfig) {
    const key = config.pendingKey
    if (key && pendingMap.has(key)) {
      const cancel = pendingMap.get(key)
      cancel && cancel(key.toString())
      pendingMap.delete(key)
    }
  }

  removeAllPending() {
    pendingMap.forEach((cancel) => {
      cancel && isFunction(cancel) && cancel()
    })
    pendingMap.clear()
  }

  reset(): void {
    pendingMap = new Map<symbol, Canceler>()
  }
}
