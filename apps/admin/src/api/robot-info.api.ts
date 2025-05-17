import { $http } from '@seed/http'

export const apiGetRobotInfo = (params?: { robotId?: number }) => {
  return $http.request('robot/RobotInfo', 'POST', params)
}
