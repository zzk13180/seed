/**
 * @file 机器人信息 API
 * @description 机器人相关接口（暂使用 fetch 直接调用，待后端路由迁移到 Hono 后切换为 RPC）
 * @module api/robot-info.api
 */

interface RobotInfo {
  id: number
  name: string
  status: string
}

interface GetRobotInfoParams {
  robotId?: number
  [key: string]: unknown
}

function isRobotInfo(data: unknown): data is RobotInfo {
  if (typeof data !== 'object' || data === null) {
    return false
  }

  return (
    'id' in data &&
    'name' in data &&
    'status' in data &&
    typeof data.id === 'number' &&
    typeof data.name === 'string' &&
    typeof data.status === 'string'
  )
}

export const apiGetRobotInfo = async (params?: GetRobotInfoParams): Promise<RobotInfo> => {
  const url = new URL('/api/robot/robot-info', globalThis.location.origin)
  if (params) {
    for (const [key, value] of Object.entries(params)) {
      if (value !== undefined && value !== null) {
        const normalizedValue =
          typeof value === 'string' || typeof value === 'number' || typeof value === 'boolean'
            ? String(value)
            : JSON.stringify(value)
        url.searchParams.set(key, normalizedValue)
      }
    }
  }
  const res = await fetch(url, { credentials: 'include' })
  if (!res.ok) throw new Error(`HTTP Error ${res.status}`)
  const data: unknown = await res.json()
  if (isRobotInfo(data)) {
    return data
  }
  throw new Error('机器人信息响应格式不正确')
}
