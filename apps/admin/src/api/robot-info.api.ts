import { $http } from '@seed/http'

interface RobotInfo {
  id: number
  name: string
  status: string
}

interface GetRobotInfoParams {
  robotId?: number
}

export const apiGetRobotInfo = (params?: GetRobotInfoParams): Promise<RobotInfo> => {
  return $http.get<RobotInfo>('/robot/robot-info', params)
}
