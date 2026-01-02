import { describe, expect, it } from 'vitest'
import {
  addTime,
  diffTime,
  endOfDay,
  formatDate,
  formatInTimeZone,
  isSameDay,
  isValidDate,
  relativeTime,
  startOfDay,
  toTimeZone,
} from '../date.util'

describe('date.util', () => {
  describe('formatDate', () => {
    it('should format date with default format', () => {
      const date = new Date('2025-01-12T10:30:45')
      expect(formatDate(date)).toBe('2025-01-12 10:30:45')
    })

    it('should format date with custom format', () => {
      const date = new Date('2025-01-12T10:30:45')
      // date-fns 使用 yyyy/MM/dd 格式
      expect(formatDate(date, 'yyyy/MM/dd')).toBe('2025/01/12')
    })

    it('should handle timestamp', () => {
      const timestamp = new Date('2025-01-12T10:30:45').getTime()
      expect(formatDate(timestamp, 'yyyy-MM-dd')).toBe('2025-01-12')
    })

    it('should return empty string for invalid date', () => {
      expect(formatDate('invalid')).toBe('')
    })
  })

  describe('relativeTime', () => {
    it('should return relative time for recent time', () => {
      const date = new Date(Date.now() - 30 * 1000) // 30 seconds ago
      const result = relativeTime(date)
      // date-fns 使用中文本地化，返回类似 "不到 1 分钟前"
      expect(result).toContain('前')
    })

    it('should return minutes ago', () => {
      const date = new Date(Date.now() - 5 * 60 * 1000) // 5 minutes ago
      const result = relativeTime(date)
      expect(result).toContain('分钟')
    })

    it('should return hours ago', () => {
      const date = new Date(Date.now() - 3 * 60 * 60 * 1000) // 3 hours ago
      const result = relativeTime(date)
      expect(result).toContain('小时')
    })

    it('should return days ago for yesterday', () => {
      const date = new Date(Date.now() - 24 * 60 * 60 * 1000) // 1 day ago
      const result = relativeTime(date)
      expect(result).toContain('天')
    })
  })

  describe('isSameDay', () => {
    it('should return true for same day', () => {
      const date1 = new Date('2025-01-12T10:00:00')
      const date2 = new Date('2025-01-12T23:59:59')
      expect(isSameDay(date1, date2)).toBe(true)
    })

    it('should return false for different days', () => {
      const date1 = new Date('2025-01-12T10:00:00')
      const date2 = new Date('2025-01-13T10:00:00')
      expect(isSameDay(date1, date2)).toBe(false)
    })
  })

  describe('startOfDay', () => {
    it('should return start of day', () => {
      const date = new Date('2025-01-12T15:30:45')
      const start = startOfDay(date)
      expect(start.getHours()).toBe(0)
      expect(start.getMinutes()).toBe(0)
      expect(start.getSeconds()).toBe(0)
    })
  })

  describe('endOfDay', () => {
    it('should return end of day', () => {
      const date = new Date('2025-01-12T10:30:45')
      const end = endOfDay(date)
      expect(end.getHours()).toBe(23)
      expect(end.getMinutes()).toBe(59)
      expect(end.getSeconds()).toBe(59)
    })
  })

  describe('addTime', () => {
    it('should add days', () => {
      const date = new Date('2025-01-12T10:00:00')
      const result = addTime(date, 5, 'day')
      expect(result.getDate()).toBe(17)
    })

    it('should subtract days', () => {
      const date = new Date('2025-01-12T10:00:00')
      const result = addTime(date, -5, 'day')
      expect(result.getDate()).toBe(7)
    })

    it('should add months', () => {
      const date = new Date('2025-01-12T10:00:00')
      const result = addTime(date, 2, 'month')
      expect(result.getMonth()).toBe(2) // March (0-indexed)
    })

    it('should add years', () => {
      const date = new Date('2025-01-12T10:00:00')
      const result = addTime(date, 1, 'year')
      expect(result.getFullYear()).toBe(2026)
    })
  })

  describe('diffTime', () => {
    it('should calculate difference in days', () => {
      const date1 = new Date('2025-01-12T10:00:00')
      const date2 = new Date('2025-01-17T10:00:00')
      expect(diffTime(date1, date2, 'day')).toBe(5)
    })

    it('should calculate difference in hours', () => {
      const date1 = new Date('2025-01-12T10:00:00')
      const date2 = new Date('2025-01-12T15:00:00')
      expect(diffTime(date1, date2, 'hour')).toBe(5)
    })
  })

  describe('isValidDate', () => {
    it('should return true for valid Date object', () => {
      expect(isValidDate(new Date())).toBe(true)
    })

    it('should return true for valid date string', () => {
      expect(isValidDate('2025-01-12')).toBe(true)
    })

    it('should return true for valid timestamp', () => {
      expect(isValidDate(Date.now())).toBe(true)
    })

    it('should return false for invalid date', () => {
      expect(isValidDate('invalid')).toBe(false)
    })

    it('should return false for null/undefined', () => {
      expect(isValidDate(null)).toBe(false)
      expect(isValidDate(undefined)).toBe(false)
    })
  })

  describe('formatInTimeZone', () => {
    it('should format date in specified timezone', () => {
      // UTC 时间
      const date = new Date('2025-01-12T00:00:00Z')
      const result = formatInTimeZone(date, 'Asia/Shanghai', 'yyyy-MM-dd HH:mm')
      // 上海时区 UTC+8
      expect(result).toBe('2025-01-12 08:00')
    })
  })

  describe('toTimeZone', () => {
    it('should create TZDate with specified timezone', () => {
      const date = new Date('2025-01-12T12:00:00Z')
      const tzDate = toTimeZone(date, 'America/New_York')
      expect(tzDate).toBeDefined()
      expect(tzDate.timeZone).toBe('America/New_York')
    })
  })
})
