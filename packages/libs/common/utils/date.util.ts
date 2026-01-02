/**
 * 统一的时间处理工具
 * 基于 date-fns + @date-fns/tz，支持 Tree-shaking，轻量高效
 */

import {
  format as dateFnsFormat,
  formatDistanceToNow,
  isSameDay as dateFnsIsSameDay,
  startOfDay as dateFnsStartOfDay,
  endOfDay as dateFnsEndOfDay,
  addYears,
  addMonths,
  addDays,
  addHours,
  addMinutes,
  addSeconds,
  differenceInDays,
  differenceInHours,
  differenceInMinutes,
  differenceInSeconds,
  differenceInMilliseconds,
  isValid,
  parseISO,
} from 'date-fns'
import { zhCN } from 'date-fns/locale'
import { TZDate, tz } from '@date-fns/tz'

// 重新导出 TZDate 和 tz 以便外部使用时区功能
export { TZDate, tz }

/**
 * 将输入转换为 Date 对象
 */
function toDate(date: Date | number | string): Date {
  if (date instanceof Date) {
    return date
  }
  if (typeof date === 'number') {
    return new Date(date)
  }
  // 尝试解析 ISO 格式字符串
  const parsed = parseISO(date)
  if (isValid(parsed)) {
    return parsed
  }
  return new Date(date)
}

/**
 * 日期格式化
 *
 * @param date 日期对象、时间戳或日期字符串
 * @param formatStr 格式化模板，使用 date-fns 格式：
 *   - yyyy: 四位年份
 *   - MM: 两位月份
 *   - dd: 两位日期
 *   - HH: 两位小时（24小时制）
 *   - mm: 两位分钟
 *   - ss: 两位秒数
 *   - 更多格式见: https://date-fns.org/docs/format
 * @param options 可选配置，包含 locale 和 timeZone
 * @returns 格式化后的日期字符串
 */
export function formatDate(
  date: Date | number | string,
  formatStr = 'yyyy-MM-dd HH:mm:ss',
  options?: { locale?: typeof zhCN; timeZone?: string },
): string {
  const d = toDate(date)

  if (!isValid(d)) {
    return ''
  }

  // 如果指定了时区，使用 TZDate
  if (options?.timeZone) {
    const tzDate = new TZDate(d, options.timeZone)
    return dateFnsFormat(tzDate, formatStr, { locale: options?.locale ?? zhCN })
  }

  return dateFnsFormat(d, formatStr, { locale: options?.locale ?? zhCN })
}

/**
 * 获取相对时间描述
 *
 * @param date 日期对象、时间戳或日期字符串
 * @param options 可选配置
 * @returns 相对时间描述，如 "刚刚"、"5分钟前"、"昨天"
 */
export function relativeTime(
  date: Date | number | string,
  options?: { addSuffix?: boolean; locale?: typeof zhCN },
): string {
  const d = toDate(date)

  if (!isValid(d)) {
    return ''
  }

  return formatDistanceToNow(d, {
    addSuffix: options?.addSuffix ?? true,
    locale: options?.locale ?? zhCN,
  })
}

/**
 * 判断是否为同一天
 */
export function isSameDay(date1: Date | number | string, date2: Date | number | string): boolean {
  return dateFnsIsSameDay(toDate(date1), toDate(date2))
}

/**
 * 获取日期的开始时间（00:00:00）
 */
export function startOfDay(date: Date | number | string): Date {
  return dateFnsStartOfDay(toDate(date))
}

/**
 * 获取日期的结束时间（23:59:59.999）
 */
export function endOfDay(date: Date | number | string): Date {
  return dateFnsEndOfDay(toDate(date))
}

/**
 * 日期加减
 *
 * @param date 基准日期
 * @param amount 增减数量
 * @param unit 单位：'day' | 'month' | 'year' | 'hour' | 'minute' | 'second'
 * @returns 新的日期对象
 */
export function addTime(
  date: Date | number | string,
  amount: number,
  unit: 'day' | 'month' | 'year' | 'hour' | 'minute' | 'second',
): Date {
  const d = toDate(date)

  switch (unit) {
    case 'year':
      return addYears(d, amount)
    case 'month':
      return addMonths(d, amount)
    case 'day':
      return addDays(d, amount)
    case 'hour':
      return addHours(d, amount)
    case 'minute':
      return addMinutes(d, amount)
    case 'second':
      return addSeconds(d, amount)
    default:
      return d
  }
}

/**
 * 获取两个日期之间的差值
 *
 * @param date1 第一个日期
 * @param date2 第二个日期
 * @param unit 单位：'day' | 'hour' | 'minute' | 'second' | 'millisecond'
 * @returns 差值（绝对值）
 */
export function diffTime(
  date1: Date | number | string,
  date2: Date | number | string,
  unit: 'day' | 'hour' | 'minute' | 'second' | 'millisecond' = 'day',
): number {
  const d1 = toDate(date1)
  const d2 = toDate(date2)

  switch (unit) {
    case 'day':
      return Math.abs(differenceInDays(d1, d2))
    case 'hour':
      return Math.abs(differenceInHours(d1, d2))
    case 'minute':
      return Math.abs(differenceInMinutes(d1, d2))
    case 'second':
      return Math.abs(differenceInSeconds(d1, d2))
    case 'millisecond':
      return Math.abs(differenceInMilliseconds(d1, d2))
    default:
      return Math.abs(differenceInMilliseconds(d1, d2))
  }
}

/**
 * 判断日期是否有效
 */
export function isValidDate(date: unknown): boolean {
  if (date instanceof Date) {
    return isValid(date)
  }
  if (typeof date === 'string') {
    const parsed = parseISO(date)
    if (isValid(parsed)) {
      return true
    }
    return isValid(new Date(date))
  }
  if (typeof date === 'number') {
    return isValid(new Date(date))
  }
  return false
}

/**
 * 在指定时区格式化日期
 *
 * @param date 日期对象、时间戳或日期字符串
 * @param timeZone 时区标识符，如 'Asia/Shanghai', 'America/New_York'
 * @param formatStr 格式化模板
 * @returns 格式化后的日期字符串
 */
export function formatInTimeZone(
  date: Date | number | string,
  timeZone: string,
  formatStr = 'yyyy-MM-dd HH:mm:ss',
): string {
  return formatDate(date, formatStr, { timeZone })
}

/**
 * 创建指定时区的日期
 *
 * @param date 日期对象、时间戳或日期字符串
 * @param timeZone 时区标识符
 * @returns TZDate 对象
 */
export function toTimeZone(date: Date | number | string, timeZone: string): TZDate {
  return new TZDate(toDate(date), timeZone)
}

// 导出 locale 以便外部使用
export { zhCN } from 'date-fns/locale'
