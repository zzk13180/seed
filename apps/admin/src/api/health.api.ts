import { $http } from '@seed/http'
import type { ApiResponse, HealthCheckResult } from './types'

/**
 * 健康检查 API
 */

/**
 * 检查服务健康状态
 */
export const apiHealthCheck = async (): Promise<HealthCheckResult> => {
  const response = await $http.get<ApiResponse<HealthCheckResult>>('/health')
  return response.data
}

/**
 * 检查服务存活状态
 */
export const apiLivenessCheck = async (): Promise<HealthCheckResult> => {
  const response = await $http.get<ApiResponse<HealthCheckResult>>('/health/liveness')
  return response.data
}
