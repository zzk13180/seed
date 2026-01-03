/**
 * API 类型定义
 *
 * 从 @seed/api-types 重新导出共享类型
 */

export { UserStatus } from '@seed/api-types/enums'
export type {
  ApiResponse,
  IPageRequest,
  IPageResult,
  IUserVo,
  IUserQueryDto,
  IUserCreateDto,
  IUserUpdateDto,
  ILoginDto,
  ILoginVo,
} from '@seed/api-types'

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
