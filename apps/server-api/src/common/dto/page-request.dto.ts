import { ApiPropertyOptional } from '@nestjs/swagger'
import { Type } from 'class-transformer'
import { IsInt, IsOptional, Min, Max, IsString, IsIn } from 'class-validator'
import { buildOrderClause, sanitizeIdentifier } from '../utils/sql.utils'

/**
 * 分页请求 DTO
 *
 * 提供统一的分页查询参数，支持排序和关键字搜索
 */
export class PageRequestDto {
  @ApiPropertyOptional({ description: '页码（从 1 开始）', minimum: 1, default: 1 })
  @IsOptional()
  @Type(() => Number)
  @IsInt()
  @Min(1)
  page: number = 1

  @ApiPropertyOptional({ description: '每页数量', minimum: 1, maximum: 100, default: 10 })
  @IsOptional()
  @Type(() => Number)
  @IsInt()
  @Min(1)
  @Max(100)
  pageSize: number = 10

  @ApiPropertyOptional({ description: '排序字段（需在允许的字段列表中）' })
  @IsOptional()
  @IsString()
  orderBy?: string

  @ApiPropertyOptional({ description: '排序方向', enum: ['ASC', 'DESC'], default: 'DESC' })
  @IsOptional()
  @IsIn(['ASC', 'DESC'])
  orderDirection: 'ASC' | 'DESC' = 'DESC'

  @ApiPropertyOptional({ description: '关键字搜索' })
  @IsOptional()
  @IsString()
  keyword?: string

  /**
   * 获取跳过的数量（用于 SQL OFFSET）
   */
  getSkip(): number {
    return (this.page - 1) * this.pageSize
  }

  /**
   * 获取每页数量（用于 SQL LIMIT）
   */
  getTake(): number {
    return this.pageSize
  }

  /**
   * 是否升序排列
   */
  isAsc(): boolean {
    return this.orderDirection === 'ASC'
  }

  /**
   * 获取安全的排序字段
   * @param allowedFields 允许排序的字段列表
   * @param defaultField 默认排序字段
   */
  getSafeOrderBy(allowedFields: string[], defaultField = 'createdAt'): string {
    const sanitized = sanitizeIdentifier(this.orderBy || '')
    if (sanitized && allowedFields.includes(sanitized)) {
      return sanitized
    }
    return defaultField
  }

  /**
   * 构建 TypeORM 排序对象
   * @param allowedFields 允许排序的字段列表
   * @param defaultField 默认排序字段
   */
  getOrderClause(
    allowedFields: string[],
    defaultField = 'createdAt',
  ): Record<string, 'ASC' | 'DESC'> {
    return buildOrderClause(this.orderBy, this.orderDirection, allowedFields, defaultField)
  }
}
