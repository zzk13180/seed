import { PageRequestDto } from './page-request.dto'

describe('PageRequestDto', () => {
  describe('default values', () => {
    it('should have default page of 1', () => {
      const dto = new PageRequestDto()
      expect(dto.page).toBe(1)
    })

    it('should have default pageSize of 10', () => {
      const dto = new PageRequestDto()
      expect(dto.pageSize).toBe(10)
    })

    it('should have default orderDirection of DESC', () => {
      const dto = new PageRequestDto()
      expect(dto.orderDirection).toBe('DESC')
    })

    it('should have undefined orderBy by default', () => {
      const dto = new PageRequestDto()
      expect(dto.orderBy).toBeUndefined()
    })
  })

  describe('getSkip', () => {
    it('should return 0 for first page', () => {
      const dto = new PageRequestDto()
      dto.page = 1
      dto.pageSize = 10

      expect(dto.getSkip()).toBe(0)
    })

    it('should calculate skip correctly for page 2', () => {
      const dto = new PageRequestDto()
      dto.page = 2
      dto.pageSize = 10

      expect(dto.getSkip()).toBe(10)
    })

    it('should calculate skip correctly for page 5 with 20 items per page', () => {
      const dto = new PageRequestDto()
      dto.page = 5
      dto.pageSize = 20

      expect(dto.getSkip()).toBe(80)
    })
  })

  describe('getTake', () => {
    it('should return pageSize', () => {
      const dto = new PageRequestDto()
      dto.pageSize = 25

      expect(dto.getTake()).toBe(25)
    })
  })

  describe('isAsc', () => {
    it('should return true when orderDirection is ASC', () => {
      const dto = new PageRequestDto()
      dto.orderDirection = 'ASC'

      expect(dto.isAsc()).toBe(true)
    })

    it('should return false when orderDirection is DESC', () => {
      const dto = new PageRequestDto()
      dto.orderDirection = 'DESC'

      expect(dto.isAsc()).toBe(false)
    })
  })
})
