import { vi, describe, it, expect, beforeEach } from 'vitest'
import { Test } from '@nestjs/testing'
import { HealthCheckService, MemoryHealthIndicator, DiskHealthIndicator } from '@nestjs/terminus'
import { DRIZZLE_DB } from '../../common/database/drizzle.constants'
import { HealthService } from './health.service'
import type { TestingModule } from '@nestjs/testing'

describe('HealthService', () => {
  let service: HealthService

  const mockHealthCheckService = {
    check: vi.fn().mockResolvedValue({
      status: 'ok',
      details: {
        database: { status: 'up' },
        memory_heap: { status: 'up' },
        memory_rss: { status: 'up' },
        disk: { status: 'up' },
      },
    }),
  }

  const mockDrizzleDB = {
    execute: vi.fn().mockResolvedValue([{ '1': 1 }]),
  }

  const mockMemoryHealthIndicator = {
    checkHeap: vi.fn().mockResolvedValue({ memory_heap: { status: 'up' } }),
    checkRSS: vi.fn().mockResolvedValue({ memory_rss: { status: 'up' } }),
  }

  const mockDiskHealthIndicator = {
    checkStorage: vi.fn().mockResolvedValue({ disk: { status: 'up' } }),
  }

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        HealthService,
        { provide: HealthCheckService, useValue: mockHealthCheckService },
        { provide: DRIZZLE_DB, useValue: mockDrizzleDB },
        { provide: MemoryHealthIndicator, useValue: mockMemoryHealthIndicator },
        { provide: DiskHealthIndicator, useValue: mockDiskHealthIndicator },
      ],
    }).compile()

    service = module.get<HealthService>(HealthService)
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
