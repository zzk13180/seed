/**
 * @file 健康检查 API
 * @description 基于 Hono RPC 的健康检查接口
 * @module api/health.api
 */

import { api } from './client'

/**
 * 检查服务就绪状态
 */
export const apiHealthCheck = async () => {
  const res = await api.api.health.$get()
  return res.json()
}

/**
 * 检查服务存活状态
 */
export const apiLivenessCheck = async () => {
  const res = await api.api.health.liveness.$get()
  return res.json()
}
