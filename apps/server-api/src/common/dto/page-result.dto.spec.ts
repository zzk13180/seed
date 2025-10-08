import { PageResultDto } from './page-result.dto'

describe('PageResultDto', () => {
  describe('constructor', () => {
    it('should create a page result with all properties', () => {
      const list = [{ id: 1 }, { id: 2 }]
      const result = new PageResultDto(list, 100, 1, 10)

      expect(result.list).toEqual(list)
      expect(result.total).toBe(100)
      expect(result.page).toBe(1)
      expect(result.pageSize).toBe(10)
      expect(result.totalPages).toBe(10)
      expect(result.hasNext).toBe(true)
      expect(result.hasPrevious).toBe(false)
    })

    it('should calculate totalPages correctly', () => {
      const result = new PageResultDto([], 25, 1, 10)

      expect(result.totalPages).toBe(3) // ceil(25/10) = 3
    })

    it('should handle zero total', () => {
      const result = new PageResultDto([], 0, 1, 10)

      expect(result.totalPages).toBe(0)
      expect(result.hasNext).toBe(false)
      expect(result.hasPrevious).toBe(false)
    })

    it('should set hasNext to false on last page', () => {
      const result = new PageResultDto([], 30, 3, 10)

      expect(result.hasNext).toBe(false)
      expect(result.hasPrevious).toBe(true)
    })

    it('should set hasPrevious to true on middle page', () => {
      const result = new PageResultDto([], 50, 3, 10)

      expect(result.hasNext).toBe(true)
      expect(result.hasPrevious).toBe(true)
    })
  })

  describe('create', () => {
    it('should create a page result using static method', () => {
      const list = [{ id: 1 }]
      const result = PageResultDto.create(list, 50, 2, 20)

      expect(result.list).toEqual(list)
      expect(result.total).toBe(50)
      expect(result.page).toBe(2)
      expect(result.pageSize).toBe(20)
      expect(result.totalPages).toBe(3)
      expect(result.hasNext).toBe(true)
      expect(result.hasPrevious).toBe(true)
    })
  })
})
