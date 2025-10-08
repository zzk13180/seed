import { Test } from '@nestjs/testing'
import { HealthController } from './health.controller'
import { HealthService } from './health.service'
import type { TestingModule } from '@nestjs/testing'

describe('HealthController', () => {
  let controller: HealthController

  const mockHealthService = {
    ping: jest.fn().mockReturnValue({
      status: 'ok',
      timestamp: expect.any(String),
    }),
    check: jest.fn().mockResolvedValue({
      status: 'ok',
      details: {
        database: { status: 'up' },
        memory_heap: { status: 'up' },
        memory_rss: { status: 'up' },
        disk: { status: 'up' },
      },
    }),
  }

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      controllers: [HealthController],
      providers: [
        {
          provide: HealthService,
          useValue: mockHealthService,
        },
      ],
    }).compile()

    controller = module.get<HealthController>(HealthController)
  })

  it('should be defined', () => {
    expect(controller).toBeDefined()
  })

  describe('liveness', () => {
    it('should return simple health status', () => {
      const result = controller.liveness()

      expect(result).toHaveProperty('status', 'ok')
      expect(mockHealthService.ping).toHaveBeenCalled()
    })
  })

  describe('check', () => {
    it('should return full health status with dependencies', async () => {
      const result = await controller.check()

      expect(result).toHaveProperty('status', 'ok')
      expect(result).toHaveProperty('details')
      expect(mockHealthService.check).toHaveBeenCalled()
    })
  })
})
