/**
 * @file HTTP 拦截器配置
 * @description 配置全局请求/响应/错误拦截器，提供统一的错误处理和用户提示
 * @module core/http.interceptor
 */

import { ElMessage, ElNotification } from 'element-plus'
import { $http, type HttpError } from '@seed/http'
import { errorHandler, ErrorType, type ErrorInfo } from './error.service'
import { createLogger } from './logger.service'

const logger = createLogger('HTTP')

/**
 * 记录最后一次错误通知的时间戳，用于防止重复提示
 */
let lastErrorNotificationTime = 0
const ERROR_NOTIFICATION_COOLDOWN = 3000 // 3秒内不重复提示相同类型错误

/**
 * 记录最后一次错误类型
 */
let lastErrorType: ErrorType | null = null

/**
 * 显示错误通知给用户
 */
function showErrorNotification(errorInfo: ErrorInfo): void {
  const now = Date.now()

  // 防止短时间内重复显示相同类型的错误
  if (
    lastErrorType === errorInfo.type &&
    now - lastErrorNotificationTime < ERROR_NOTIFICATION_COOLDOWN
  ) {
    logger.debug('Skipping duplicate error notification', { type: errorInfo.type })
    return
  }

  lastErrorNotificationTime = now
  lastErrorType = errorInfo.type

  // 根据错误类型选择不同的通知方式
  switch (errorInfo.type) {
    case ErrorType.NETWORK_ERROR:
    case ErrorType.SERVICE_UNAVAILABLE:
    case ErrorType.TIMEOUT_ERROR: {
      // 网络/服务相关错误使用 Notification（右上角持久通知）
      ElNotification({
        title: errorInfo.title,
        message: errorInfo.message,
        type: 'error',
        duration: 5000,
        position: 'top-right',
      })
      break
    }

    case ErrorType.UNAUTHORIZED: {
      // 登录过期使用警告通知
      ElNotification({
        title: errorInfo.title,
        message: errorInfo.message,
        type: 'warning',
        duration: 4000,
        position: 'top-right',
      })
      break
    }

    case ErrorType.FORBIDDEN:
    case ErrorType.NOT_FOUND:
    case ErrorType.CLIENT_ERROR: {
      // 客户端错误使用 Message（顶部短暂提示）
      ElMessage.error(errorInfo.message)
      break
    }

    case ErrorType.SERVER_ERROR: {
      // 服务器错误使用 Notification
      ElNotification({
        title: errorInfo.title,
        message: errorInfo.message,
        type: 'error',
        duration: 4000,
        position: 'top-right',
      })
      break
    }

    default: {
      ElMessage.error(errorInfo.message)
    }
  }
}

/**
 * 处理需要重新登录的错误
 */
async function handleAuthError(): Promise<void> {
  // 延迟导入以避免循环依赖
  const { useUserStore } = await import('@/stores/user/user.store')

  try {
    const userStore = useUserStore()
    // 清除用户状态并跳转到登录页
    await userStore.controller.logout()
  } catch (error) {
    logger.error('Failed to handle auth error', error)
    // 如果登出失败，强制跳转到登录页
    globalThis.location.href = '/login'
  }
}

/**
 * 全局错误拦截器
 */
function setupErrorInterceptor(): () => void {
  return $http.addErrorInterceptor(async (error: HttpError) => {
    const errorInfo = errorHandler.parseError(error)

    // 记录错误日志
    logger.error('HTTP Error', {
      type: errorInfo.type,
      status: errorInfo.status,
      message: errorInfo.message,
      endpoint: error.config?.endpoint,
    })

    // 显示用户通知
    if (errorInfo.shouldNotify) {
      showErrorNotification(errorInfo)
    }

    // 处理需要重新登录的情况
    if (errorInfo.requiresReLogin) {
      await handleAuthError()
    }

    // 继续抛出错误，让调用方可以进行额外处理
    throw error
  })
}

/**
 * 请求拦截器 - 添加通用请求头等
 */
function setupRequestInterceptor(): () => void {
  return $http.addRequestInterceptor(config => {
    logger.debug('Request', { endpoint: config.endpoint, method: config.method })
    return config
  })
}

/**
 * 响应拦截器 - 处理统一响应格式
 */
function setupResponseInterceptor(): () => void {
  return $http.addResponseInterceptor((response, config) => {
    logger.debug('Response', { endpoint: config.endpoint })
    return response
  })
}

/**
 * 初始化所有 HTTP 拦截器
 * @returns 清理函数，调用后会移除所有拦截器
 */
export function setupHttpInterceptors(): () => void {
  const cleanupFunctions = [
    setupRequestInterceptor(),
    setupResponseInterceptor(),
    setupErrorInterceptor(),
  ]

  logger.info('HTTP interceptors initialized')

  return () => {
    cleanupFunctions.forEach(cleanup => cleanup())
    logger.info('HTTP interceptors cleaned up')
  }
}

/**
 * 检查服务是否可用
 * 用于应用启动时的健康检查
 */
export async function checkServiceHealth(): Promise<boolean> {
  try {
    // 尝试请求健康检查接口
    await $http.get('/health')
    return true
  } catch (error) {
    const errorInfo = errorHandler.parseError(error)

    if (
      errorInfo.type === ErrorType.NETWORK_ERROR ||
      errorInfo.type === ErrorType.SERVICE_UNAVAILABLE
    ) {
      ElNotification({
        title: '服务连接失败',
        message: '无法连接到后端服务，部分功能可能无法使用',
        type: 'warning',
        duration: 0, // 不自动关闭
        position: 'top-right',
      })
    }

    return false
  }
}
