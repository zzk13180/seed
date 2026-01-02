import { vi, describe, it, expect, beforeEach, type Mock, type Mocked } from 'vitest'
import { PermissionsGuard } from './permissions.guard'
import type { ExecutionContext } from '@nestjs/common'
import type { Reflector } from '@nestjs/core'

describe('PermissionsGuard', () => {
  let guard: PermissionsGuard
  let reflector: Mocked<Reflector>

  beforeEach(() => {
    reflector = {
      getAllAndOverride: vi.fn(),
    } as any
    guard = new PermissionsGuard(reflector)
  })

  const createMockExecutionContext = (user: unknown): ExecutionContext =>
    ({
      switchToHttp: () => ({
        getRequest: () => ({ user }),
      }),
      getHandler: () => ({}),
      getClass: () => ({}),
    }) as unknown as ExecutionContext

  it('should be defined', () => {
    expect(guard).toBeDefined()
  })

  describe('canActivate', () => {
    it('should return true when no permissions are required', () => {
      reflector.getAllAndOverride.mockReturnValue(undefined)
      const context = createMockExecutionContext({ permissions: ['user:read'] })

      expect(guard.canActivate(context)).toBe(true)
    })

    it('should return true when user has required permission', () => {
      reflector.getAllAndOverride.mockReturnValue(['user:read'])
      const context = createMockExecutionContext({
        permissions: ['user:read', 'user:write'],
      })

      expect(guard.canActivate(context)).toBe(true)
    })

    it('should return false when user does not have required permission', () => {
      reflector.getAllAndOverride.mockReturnValue(['user:delete'])
      const context = createMockExecutionContext({
        permissions: ['user:read', 'user:write'],
      })

      expect(() => guard.canActivate(context)).toThrow()
    })

    it('should return true when user has any of the required permissions', () => {
      reflector.getAllAndOverride.mockReturnValue(['user:write', 'user:delete'])
      const context = createMockExecutionContext({
        permissions: ['user:write'],
      })

      expect(guard.canActivate(context)).toBe(true)
    })

    it('should return false when user has no permissions', () => {
      reflector.getAllAndOverride.mockReturnValue(['user:read'])
      const context = createMockExecutionContext({ permissions: undefined })

      expect(() => guard.canActivate(context)).toThrow()
    })
  })
})
