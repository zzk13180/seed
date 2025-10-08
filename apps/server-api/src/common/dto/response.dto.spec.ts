import { ResponseDto } from './response.dto'

describe('ResponseDto', () => {
  describe('constructor', () => {
    it('should create a response with all properties', () => {
      const response = new ResponseDto(200, 'Success', { id: 1 })

      expect(response.code).toBe(200)
      expect(response.message).toBe('Success')
      expect(response.data).toEqual({ id: 1 })
      expect(response.timestamp).toBeDefined()
      expect(typeof response.timestamp).toBe('number')
    })
  })

  describe('ok', () => {
    it('should create a success response with data', () => {
      const data = { id: 1, name: 'test' }
      const response = ResponseDto.ok(data)

      expect(response.code).toBe(200)
      expect(response.message).toBe('Success')
      expect(response.data).toEqual(data)
    })

    it('should create a success response with custom message', () => {
      const response = ResponseDto.ok(null, 'Created successfully')

      expect(response.code).toBe(200)
      expect(response.message).toBe('Created successfully')
      expect(response.data).toBeNull()
    })

    it('should create a success response without data', () => {
      const response = ResponseDto.ok()

      expect(response.code).toBe(200)
      expect(response.data).toBeNull()
    })
  })

  describe('error', () => {
    it('should create an error response with default values', () => {
      const response = ResponseDto.error()

      expect(response.code).toBe(500)
      expect(response.message).toBe('Internal Server Error')
      expect(response.data).toBeNull()
    })

    it('should create an error response with custom code and message', () => {
      const response = ResponseDto.error(400, 'Bad Request')

      expect(response.code).toBe(400)
      expect(response.message).toBe('Bad Request')
    })

    it('should create an error response with data', () => {
      const errors = [{ field: 'username', message: 'required' }]
      const response = ResponseDto.error(400, 'Validation Error', errors)

      expect(response.code).toBe(400)
      expect(response.data).toEqual(errors)
    })
  })
})
