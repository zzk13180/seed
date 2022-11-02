import qs from 'qs'
import type { CreateAxiosOptions } from '../axios'
import type { AxiosRequestConfig, AxiosResponse } from 'axios'

let nextRequestId = 0

export class Jsonp {
  private static instance: Jsonp
  private readonly resolvedPromise = Promise.resolve()
  private readonly config: CreateAxiosOptions
  private constructor(config: CreateAxiosOptions) {
    this.config = config
  }

  public static getInstance(config: CreateAxiosOptions = {}): Jsonp {
    if (!Jsonp.instance) {
      Jsonp.instance = new Jsonp(config)
    }
    return Jsonp.instance
  }

  private nextCallback(): string {
    return `pms_jsonp_callback_${nextRequestId++}`
  }

  public async request<T = any, R = AxiosResponse<T>>(
    requestConfig: AxiosRequestConfig,
  ): Promise<R> {
    let timeoutId: any
    let canceler = () => {}
    if (this.config.timeout) {
      timeoutId = setTimeout(() => canceler(), this.config.timeout)
    }
    const done = () => timeoutId && clearTimeout(timeoutId)
    const res = await new Promise<R>((resolve, reject) => {
      canceler = this.handle(requestConfig, resolve, reject, done)
    })
    return res
  }

  private handle(requestConfig, resolve, reject, done) {
    const params = qs.stringify(requestConfig.params, { arrayFormat: 'brackets' })
    const callbackKey = 'callback'
    // generate the callback name
    const callbackName = this.nextCallback()
    const url = `${
      this.config.baseURL ? this.config.baseURL + requestConfig.url : requestConfig.url
    }?${callbackKey}=${callbackName}${params ? `&${params}` : ''}`
    // Construct the <script> tag
    const node = document.createElement('script')
    // point src at the URL
    node.src = url
    // The response object
    let body: any | null = null
    // Whether the response callback has been called.
    let finished = false
    // Whether the request has been cancelled
    let cancelled = false
    // the response callback from the window
    window[callbackName] = (data?: any) => {
      // Data has been received from the JSONP script.
      // Firstly, delete this callback.
      delete window[callbackName]
      // Next, make sure the request wasn't cancelled in the meantime.
      if (cancelled) {
        return
      }
      // Set state to indicate data was received.
      body = data
      finished = true
    }
    // Cleanup the page.
    const cleanup = () => {
      // clearTimeout
      done()
      // Remove the <script> tag if it's still on the page.
      if (node.parentNode) {
        node.parentNode.removeChild(node)
      }
      // Remove the response callback from the window
      delete window[callbackName]
    }
    // success callback
    const onLoad = () => {
      // Do nothing if the request has been cancelled.
      if (cancelled) {
        return
      }
      // wrap it in an extra Promise, to ensure the microtask
      this.resolvedPromise.then(() => {
        cleanup()
        // Check whether the response callback has run.
        if (!finished) {
          reject({
            response: {},
            code: 400,
            message: 'JSONP 请求失败',
            config: requestConfig,
          })
        }
        const response: AxiosResponse = {
          data: body,
          status: 200,
          statusText: 'OK',
          headers: {},
          config: requestConfig,
        }
        resolve(response)
      })
    }
    //  error callback
    const onError: any = (error: Error) => {
      done()
      if (cancelled) {
        return
      }
      cleanup()
      reject({
        response: error,
        code: 400,
        message: 'JSONP 请求失败',
        config: requestConfig,
      })
    }
    node.addEventListener('load', onLoad)
    node.addEventListener('error', onError)
    document.body.appendChild(node)
    // Cancellation handler.
    return () => {
      // Track the cancellation so event listeners won't do anything even if already scheduled.
      cancelled = true
      // Remove the event listeners so they won't run if the events later fire.
      node.removeEventListener('load', onLoad)
      node.removeEventListener('error', onError)
      // And finally, clean up the page.
      cleanup()
      reject({
        response: {},
        code: 400,
        message: 'JSONP 请求超时',
        config: requestConfig,
      })
    }
  }
}
