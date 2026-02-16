/**
 * @file HTTP 错误处理
 * @description 全局 fetch 错误处理，配合 Hono RPC 客户端使用
 * @module core/http.interceptor
 *
 * 迁移说明：
 * - 旧方案: @seed/http 的 HttpClient 拦截器
 * - 新方案: Hono RPC 的 hc() 客户端 + Better Auth cookie-session
 * - Hono RPC 使用原生 fetch，无拦截器机制
 * - 错误处理通过 onError 回调或在 API 调用处 try/catch
 */

import { ElMessage, ElNotification } from 'element-plus'
import { createLogger } from './logger.service'

const logger = createLogger('HTTP')

/**
 * 记录最后一次错误通知的时间戳，用于防止重复提示
 */
let lastErrorNotificationTime = 0
const ERROR_NOTIFICATION_COOLDOWN = 3000

/**
 * 记录最后一次错误状态码
 */
let lastErrorStatus = 0

/**
 * 处理 HTTP 响应错误
 *
 * 供 API 调用层在 catch 中使用
 */
export function handleHttpError(error: unknown): void {
  const now = Date.now()

  // 网络错误
  if (error instanceof TypeError && error.message.includes('fetch')) {
    if (now - lastErrorNotificationTime > ERROR_NOTIFICATION_COOLDOWN || lastErrorStatus !== 0) {
      lastErrorNotificationTime = now
      lastErrorStatus = 0
      ElNotification({
        title: '网络错误',
        message: '无法连接到服务器，请检查网络连接',
        type: 'error',
        duration: 5000,
        position: 'top-right',
      })
    }
    return
  }

  // 解析 Response 错误
  if (error instanceof Response) {
    const status = error.status

    if (
      lastErrorStatus === status &&
      now - lastErrorNotificationTime < ERROR_NOTIFICATION_COOLDOWN
    ) {
      return
    }

    lastErrorNotificationTime = now
    lastErrorStatus = status

    switch (status) {
      case 401: {
        ElNotification({
          title: '登录已过期',
          message: '请重新登录',
          type: 'warning',
          duration: 4000,
          position: 'top-right',
        })
        // 延迟导入避免循环依赖
        void (async () => {
          const { useUserStore } = await import('@/stores/user/user.store')
          const userStore = useUserStore()
          await userStore.controller.logout()
          await userStore.controller.navigateToLogin()
        })().catch(() => {
          globalThis.location.href = '/login'
        })
        break
      }
      case 403: {
        ElMessage.error('没有权限执行此操作')
        break
      }
      case 404: {
        ElMessage.error('请求的资源不存在')
        break
      }
      case 429: {
        ElMessage.warning('请求过于频繁，请稍后重试')
        break
      }
      default: {
        if (status >= 500) {
          ElNotification({
            title: '服务器错误',
            message: '服务器内部错误，请稍后重试',
            type: 'error',
            duration: 4000,
            position: 'top-right',
          })
        } else {
          ElMessage.error(`请求失败 (${status})`)
        }
      }
    }
    return
  }

  // 其他错误
  if (error instanceof Error) {
    logger.error('Unexpected error', error)
    ElMessage.error(error.message || '未知错误')
  }
}

/**
 * 初始化全局错误处理
 *
 * Better Auth + Hono RPC 模式下，不再需要 $http 拦截器
 * 此函数保持空实现以兼容现有启动流程
 */
export function setupHttpInterceptors(): () => void {
  logger.info('HTTP error handlers initialized (Hono RPC + Better Auth mode)')
  return () => {
    logger.info('HTTP error handlers cleaned up')
  }
}

/**
 * 检查服务是否可用
 */
export async function checkServiceHealth(): Promise<boolean> {
  try {
    const res = await fetch('/api/health/liveness', { credentials: 'include' })
    return res.ok
  } catch {
    ElNotification({
      title: '服务连接失败',
      message: '无法连接到后端服务，部分功能可能无法使用',
      type: 'warning',
      duration: 0,
      position: 'top-right',
    })
    return false
  }
}
