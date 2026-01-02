import { AsyncLocalStorage } from 'node:async_hooks'

/**
 * 审计上下文存储
 * 使用 AsyncLocalStorage 确保每个请求有独立的上下文，解决并发问题
 */
export interface AuditContext {
  userId: number | null
}

export const auditStorage = new AsyncLocalStorage<AuditContext>()

/**
 * 审计工具类
 *
 * 用于获取和管理当前请求的用户 ID，用于审计字段的自动填充
 *
 * 注意：使用 AsyncLocalStorage 确保在高并发环境下每个请求的用户 ID 隔离
 */
export const AuditSubscriber = {
  /**
   * 在异步上下文中运行回调
   */
  run<T>(userId: number | null, callback: () => T): T {
    return auditStorage.run({ userId }, callback)
  },

  /**
   * 获取当前用户 ID
   */
  getCurrentUserId(): number | null {
    const context = auditStorage.getStore()
    return context?.userId ?? null
  },
}
