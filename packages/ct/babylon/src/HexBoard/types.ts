export interface Item {
  id: string
  u?: number
  v?: number
  lineWidth?: number
  borderWidth?: number
  lineColor?: string
  fillColor?: string
  borderColor?: string
  scaleLength?: number
  rotation?: number
  scaleWidth?: number
  size?: number
  angle?: number
  thickness?: number
  diameter?: number
  sides?: number
  vectorU?: number
  vectorV?: number
  isEmissive?: boolean
  vertical?: boolean
  type?: string
  img?: string
  color?: string
  backgroundColor?: string
  onClick?: Fn
  vectors?: any
}
