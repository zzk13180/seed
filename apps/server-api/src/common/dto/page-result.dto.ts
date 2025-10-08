import { ApiProperty } from '@nestjs/swagger'

/**
 * 分页响应 DTO
 */
export class PageResultDto<T> {
  static create<T>(list: T[], total: number, page: number, pageSize: number): PageResultDto<T> {
    return new PageResultDto(list, total, page, pageSize)
  }

  @ApiProperty({ description: '数据列表' })
  list: T[]

  @ApiProperty({ description: '总记录数' })
  total: number

  @ApiProperty({ description: '当前页码' })
  page: number

  @ApiProperty({ description: '每页数量' })
  pageSize: number

  @ApiProperty({ description: '总页数' })
  totalPages: number

  @ApiProperty({ description: '是否有下一页' })
  hasNext: boolean

  @ApiProperty({ description: '是否有上一页' })
  hasPrevious: boolean

  constructor(list: T[], total: number, page: number, pageSize: number) {
    this.list = list
    this.total = total
    this.page = page
    this.pageSize = pageSize
    this.totalPages = pageSize > 0 ? Math.ceil(total / pageSize) : 0
    this.hasNext = page < this.totalPages
    this.hasPrevious = page > 1
  }
}
