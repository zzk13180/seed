import { describe, it, expect } from 'vitest'
import { ResponseCode } from '../enums/response-code.enum'
import { ResponseDto } from './response.dto'

describe('ResponseDto', () => {
  describe('constructor', () => {
    it('should create a response with all properties', () => {
      const response = new ResponseDto(ResponseCode.SUCCESS, 'Success', { id: 1 })

      expect(response.code).toBe(ResponseCode.SUCCESS)
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

      expect(response.code).toBe(ResponseCode.SUCCESS)
      expect(response.message).toBe('Success')
      expect(response.data).toEqual(data)
    })

    it('should create a success response with custom message', () => {
      const response = ResponseDto.ok(null, 'Created successfully')

      expect(response.code).toBe(ResponseCode.SUCCESS)
      expect(response.message).toBe('Created successfully')
      expect(response.data).toBeNull()
    })

    it('should create a success response without data', () => {
      const response = ResponseDto.ok()

      expect(response.code).toBe(ResponseCode.SUCCESS)
      expect(response.data).toBeNull()
    })
  })

  describe('error', () => {
    it('should create an error response with default values', () => {
      const response = ResponseDto.error()

      expect(response.code).toBe(ResponseCode.ERROR)
      expect(response.message).toBe('操作失败')
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
