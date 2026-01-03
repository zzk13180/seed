import { IsArray, IsInt, ArrayMinSize, ArrayMaxSize } from 'class-validator'
import { ApiProperty } from '@nestjs/swagger'
import type { IBatchDeleteParams } from '@seed/api-types'

/**
 * 批量删除 DTO
 *
 * 实现 @seed/api-types 中的 IBatchDeleteParams 接口
 */
export class BatchDeleteDto implements IBatchDeleteParams {
  @ApiProperty({ description: '要删除的用户 ID 列表', example: [1, 2, 3] })
  @IsArray({ message: 'ids 必须是数组' })
  @IsInt({ each: true, message: 'ids 中的每个元素必须是整数' })
  @ArrayMinSize(1, { message: '至少需要一个 ID' })
  @ArrayMaxSize(100, { message: '一次最多删除 100 个' })
  ids!: number[]
}
