import { Test } from '@nestjs/testing'
import {
  HealthCheckService,
  TypeOrmHealthIndicator,
  MemoryHealthIndicator,
  DiskHealthIndicator,
} from '@nestjs/terminus'
import { HealthService } from './health.service'
import type { TestingModule } from '@nestjs/testing'

describe('HealthService', () => {
  let service: HealthService
  let healthCheckService: HealthCheckService

  const mockHealthCheckService = {
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

  const mockTypeOrmHealthIndicator = {
    pingCheck: jest.fn().mockResolvedValue({ database: { status: 'up' } }),
  }

  const mockMemoryHealthIndicator = {
    checkHeap: jest.fn().mockResolvedValue({ memory_heap: { status: 'up' } }),
    checkRSS: jest.fn().mockResolvedValue({ memory_rss: { status: 'up' } }),
  }

  const mockDiskHealthIndicator = {
    checkStorage: jest.fn().mockResolvedValue({ disk: { status: 'up' } }),
  }

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        HealthService,
        { provide: HealthCheckService, useValue: mockHealthCheckService },
        { provide: TypeOrmHealthIndicator, useValue: mockTypeOrmHealthIndicator },
        { provide: MemoryHealthIndicator, useValue: mockMemoryHealthIndicator },
        { provide: DiskHealthIndicator, useValue: mockDiskHealthIndicator },
      ],
    }).compile()

    service = module.get<HealthService>(HealthService)
    healthCheckService = module.get<HealthCheckService>(HealthCheckService)
  })

  it('should be defined', () => {
    expect(service).toBeDefined()
  })

  describe('ping', () => {
    it('should return health status with ok status', () => {
      const result = service.ping()

      expect(result).toHaveProperty('status', 'ok')
      expect(result).toHaveProperty('timestamp')
      expect(typeof result.timestamp).toBe('string')
    })

    it('should return a valid ISO timestamp', () => {
      const result = service.ping()

      // Check if timestamp is a valid ISO date string
      const timestamp = new Date(result.timestamp)
      expect(timestamp.toISOString()).toBe(result.timestamp)
    })
  })

  describe('check', () => {
    it('should call healthCheckService.check with health indicators', async () => {
      await service.check()

      expect(mockHealthCheckService.check).toHaveBeenCalled()
    })

    it('should return health check result', async () => {
      const result = await service.check()

      expect(result).toHaveProperty('status', 'ok')
      expect(result).toHaveProperty('details')
    })
  })
})
