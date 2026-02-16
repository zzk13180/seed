/**
 * SQL 工具函数
 */

/**
 * 转义 LIKE 查询中的特殊字符（%, _, \）
 *
 * @example
 * ```typescript
 * const keyword = '100%_complete'
 * const safe = escapeLikeString(keyword)
 * // → '100\%\_complete'
 * db.select().from(user).where(like(user.name, `%${safe}%`))
 * ```
 */
export function escapeLikeString(value: string): string {
  return value
    .replaceAll('\\', '\\\\')
    .replaceAll('%', String.raw`\%`)
    .replaceAll('_', String.raw`\_`)
}
