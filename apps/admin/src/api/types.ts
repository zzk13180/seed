/**
 * API 类型定义
 *
 * 从 @seed/kit 重新导出共享类型
 */

export { UserStatus } from '@seed/contracts'
export type {
  PageResult,
  UserVO,
  UserQuery as UserQueryDto,
  UserCreateDto,
  UserUpdateDto,
} from '@seed/contracts'

// ============================================================================
// Admin 特有的类型定义
// ============================================================================

/**
 * 健康检查响应
 */
export interface HealthCheckResult {
  status: 'ok' | 'error'
  info?: Record<string, unknown>
  error?: Record<string, unknown>
  details?: Record<string, unknown>
}
