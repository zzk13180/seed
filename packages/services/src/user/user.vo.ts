import type { User } from '@seed/db/schema'

/**
 * 用户视图对象 — 控制返回给前端的字段
 *
 * 隐藏内部字段（deleted、password 等），格式化时间
 */
export interface UserVO {
  id: string
  name: string
  email: string
  emailVerified: boolean
  image: string | null
  phone: string | null
  role: string
  status: number
  createdAt: string
  updatedAt: string
}

/**
 * Entity → VO 转换
 */
export function toUserVO(entity: User): UserVO {
  return {
    id: entity.id,
    name: entity.name,
    email: entity.email,
    emailVerified: entity.emailVerified,
    image: entity.image,
    phone: entity.phone,
    role: entity.role,
    status: entity.status,
    createdAt: entity.createdAt.toISOString(),
    updatedAt: entity.updatedAt.toISOString(),
  }
}
