import { plainToInstance } from 'class-transformer'
import type { ClassConstructor } from 'class-transformer'

/**
 * 通用对象转换器基类
 *
 * 提供 Entity 与 VO/DTO 之间的转换功能
 */
export abstract class BaseMapper<Entity, VO> {
  protected abstract voClass: ClassConstructor<VO>

  /**
   * Entity 转 VO
   */
  toVO(entity: Entity): VO {
    if (!entity) {
      return null as VO
    }
    return plainToInstance(this.voClass, entity, {
      excludeExtraneousValues: false, // 保留所有属性
      enableImplicitConversion: true, // 启用隐式类型转换
    })
  }

  /**
   * Entity 列表转 VO 列表
   */
  toVOList(entities: Entity[]): VO[] {
    if (!entities || entities.length === 0) {
      return []
    }
    return entities.map(entity => this.toVO(entity))
  }
}

/**
 * 简单转换工具函数
 */
export function toVO<T>(voClass: ClassConstructor<T>, source: object | null | undefined): T | null {
  if (!source) {
    return null
  }
  return plainToInstance(voClass, source, {
    excludeExtraneousValues: false,
    enableImplicitConversion: true,
  })
}

/**
 * 批量转换工具函数
 */
export function toVOList<T>(
  voClass: ClassConstructor<T>,
  sources: object[] | null | undefined,
): T[] {
  if (!sources || sources.length === 0) {
    return []
  }
  return sources.map(source =>
    plainToInstance(voClass, source, {
      excludeExtraneousValues: false,
      enableImplicitConversion: true,
    }),
  )
}
