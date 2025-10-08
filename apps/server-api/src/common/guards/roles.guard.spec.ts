import { RolesGuard } from './roles.guard'
import type { ExecutionContext } from '@nestjs/common'
import type { Reflector } from '@nestjs/core'

describe('RolesGuard', () => {
  let guard: RolesGuard
  let reflector: jest.Mocked<Reflector>

  beforeEach(() => {
    reflector = {
      getAllAndOverride: jest.fn(),
    } as any
    guard = new RolesGuard(reflector)
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
    it('should return true when no roles are required', () => {
      reflector.getAllAndOverride.mockReturnValue(undefined)
      const context = createMockExecutionContext({ roles: ['USER'] })

      expect(guard.canActivate(context)).toBe(true)
    })

    it('should return true when user has required role', () => {
      reflector.getAllAndOverride.mockReturnValue(['ADMIN'])
      const context = createMockExecutionContext({ roles: ['ADMIN', 'USER'] })

      expect(guard.canActivate(context)).toBe(true)
    })

    it('should return false when user does not have required role', () => {
      reflector.getAllAndOverride.mockReturnValue(['ADMIN'])
      const context = createMockExecutionContext({ roles: ['USER'] })

      expect(guard.canActivate(context)).toBe(false)
    })

    it('should return true when user has any of the required roles', () => {
      reflector.getAllAndOverride.mockReturnValue(['ADMIN', 'MODERATOR'])
      const context = createMockExecutionContext({ roles: ['MODERATOR'] })

      expect(guard.canActivate(context)).toBe(true)
    })

    it('should return false when user has no roles', () => {
      reflector.getAllAndOverride.mockReturnValue(['ADMIN'])
      const context = createMockExecutionContext({ roles: undefined })

      expect(guard.canActivate(context)).toBe(false)
    })
  })
})
