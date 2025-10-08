import { AsyncLocalStorage } from 'node:async_hooks'
import { EntitySubscriberInterface, EventSubscriber, InsertEvent, UpdateEvent } from 'typeorm'

/**
 * 审计上下文存储
 * 使用 AsyncLocalStorage 确保每个请求有独立的上下文，解决并发问题
 */
export interface AuditContext {
  userId: number | null
}

export const auditStorage = new AsyncLocalStorage<AuditContext>()

/**
 * 审计订阅器
 *
 * 自动填充实体的 createdBy 和 updatedBy 字段
 *
 * 注意：使用 AsyncLocalStorage 确保在高并发环境下每个请求的用户 ID 隔离
 */
@EventSubscriber()
export class AuditSubscriber implements EntitySubscriberInterface {
  /**
   * 在异步上下文中运行回调
   */
  static run<T>(userId: number | null, callback: () => T): T {
    return auditStorage.run({ userId }, callback)
  }

  /**
   * 获取当前用户 ID
   */
  static getCurrentUserId(): number | null {
    const context = auditStorage.getStore()
    return context?.userId ?? null
  }

  /**
   * 插入前自动填充 createdBy
   */
  beforeInsert(event: InsertEvent<any>): void {
    const userId = AuditSubscriber.getCurrentUserId()
    if (userId && event.entity) {
      if ('createdBy' in event.entity && !event.entity.createdBy) {
        event.entity.createdBy = userId
      }
      if ('updatedBy' in event.entity && !event.entity.updatedBy) {
        event.entity.updatedBy = userId
      }
    }
  }

  /**
   * 更新前自动填充 updatedBy
   */
  beforeUpdate(event: UpdateEvent<any>): void {
    const userId = AuditSubscriber.getCurrentUserId()
    if (userId && event.entity && 'updatedBy' in event.entity) {
      event.entity.updatedBy = userId
    }
  }
}
