import { RequestEnum, ContentTypeEnum } from '@seed/common/enums/http.enum'
import { isString } from '@seed/common/utils/is.util'
import { VAxios } from './VAxios'
import type { AxiosRequestConfig, AxiosResponse } from 'axios'

const transform: AxiosTransform = {
  /**
   * 发起请求前 处理 config
   * 此时请求还未发起
   * 处理 params
   */
  beforeRequestHook: config => {
    const params = config.params || {}
    const data = config.data || false
    if (isString(params)) {
      config.url += params
      config.params = undefined
      return config
    }
    if (config.method?.toUpperCase() === RequestEnum.GET) {
      config.params = params
      return config
    }
    if (Reflect.has(config, 'data') && config.data) {
      config.data = data
      config.params = params
    } else {
      // 非GET请求如果没有提供data，则将params视为data
      config.data = params
      config.params = undefined
    }
    return config
  },
  /**
   * 发起请求前 处理 config
   * interceptors.request.use 第一个参数
   * 处理 token
   */
  requestInterceptors: (config, options) => {
    const token = '' // getToken()
    if (token && !config.headers?.Authorization) {
      if (!config.headers) {
        config.headers = {}
      }
      config.headers.Authorization = options.authenticationScheme
        ? `${options.authenticationScheme} ${token}`
        : token
    }
    return config
  },
  /**
   * 请求返回后 处理 respone
   * interceptors.response.use 第一个参数
   */
  responseInterceptors: (respone: AxiosResponse<any>) => {
    return respone
  },
  /**
   * 请求返回后 处理 respone
   * 此时请求已完成
   */
  transformRequestHook: (respone: AxiosResponse<any>, options: RequestOptions) => {
    const { isReturnNativeResponse } = options
    if (isReturnNativeResponse) {
      return respone
    }
    return respone.data
  },
  /**
   * 请求前 错误处理
   * interceptors.request.use 第二个参数
   */
  requestInterceptorsCatch: (error: any) => {
    return error
  },
  /**
   * 请求后 错误处理
   * interceptors.response.use 第二个参数
   */
  responseInterceptorsCatch: (error: any) => {
    // const { response, code, message, config } = error || {}
    // console.log(response, code, message, config)
    return error
  },
  requestCatchHook: (error: any, options: RequestOptions) => {
    const { isTransformError } = options
    if (!isTransformError) {
      return error
    }
    return error // Promise.reject(error)
  },
}

const defaultConfig: Partial<CreateAxiosOptions> = {
  transform,
  authenticationScheme: 'Bearer', // Bearer Authentication#authentication_schemes
  timeout: 2 * 60 * 1000, // 请求超时时间
  headers: { 'Content-Type': ContentTypeEnum.JSON },
  baseURL: (import.meta.env.VITE_GLOB_APP_MOCK_APIURL as string | undefined) || '',
  requestOptions: {
    errorMessageMode: 'message', // 错误消息提示类型
    isReturnNativeResponse: false, // 是否返回原生响应头
    isTransformResponse: false, // 是否对返回的数据进行处理
    isTransformError: false, // 是否对返回的错误进行处理
  },
}

export const $http = new VAxios(defaultConfig)

export type ErrorMessageMode = 'none' | 'modal' | 'message' | undefined

export interface RequestOptions {
  errorMessageMode?: ErrorMessageMode
  isReturnNativeResponse?: boolean
  isTransformResponse?: boolean
  isTransformError?: boolean
}

export interface CreateAxiosOptions extends AxiosRequestConfig {
  authenticationScheme?: string
  transform?: AxiosTransform
  requestOptions?: RequestOptions
}

export abstract class AxiosTransform {
  /**
   * @description: Process configuration before request
   * @description: Process configuration before request
   */
  beforeRequestHook?: (
    config: AxiosRequestConfig,
    options: RequestOptions,
  ) => AxiosRequestConfig

  /**
   * @description: Request successfully processed
   */
  transformRequestHook?: (res: AxiosResponse<any>, options: RequestOptions) => any

  /**
   * @description: 请求失败处理
   */
  requestCatchHook?: (e: Error, options: RequestOptions) => Promise<any>

  /**
   * @description: 请求之前的拦截器
   */
  requestInterceptors?: (
    config: AxiosRequestConfig,
    options: CreateAxiosOptions,
  ) => AxiosRequestConfig

  /**
   * @description: 请求之后的拦截器
   */
  responseInterceptors?: (res: AxiosResponse<any>) => AxiosResponse<any>

  /**
   * @description: 请求之前的拦截器错误处理
   */
  requestInterceptorsCatch?: (error: Error) => void

  /**
   * @description: 请求之后的拦截器错误处理
   */
  responseInterceptorsCatch?: (error: Error) => void
}

export interface UploadFileParams {
  // Other parameters
  data?: Recordable
  name?: string
  file: File | Blob
  filename?: string
  [key: string]: any
}
