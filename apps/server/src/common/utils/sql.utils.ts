/**
 * 转义 SQL LIKE 查询中的特殊字符
 *
 * 防止 SQL 注入攻击，转义 %, _, \ 等特殊字符
 *
 * @param value - 要转义的字符串
 * @returns 转义后的字符串
 */
export function escapeLikeString(value: string): string {
  if (!value) {
    return value
  }

  // 转义 LIKE 查询中的特殊字符
  return value
    .replaceAll('\\', '\\\\') // 先转义反斜杠
    .replaceAll('%', String.raw`\%`) // 转义百分号
    .replaceAll('_', String.raw`\_`) // 转义下划线
}

/**
 * 清理并验证标识符（表名、列名等）
 *
 * 只允许字母、数字、下划线
 *
 * @param identifier - 标识符
 * @returns 清理后的标识符，如果无效则返回 null
 */
export function sanitizeIdentifier(identifier: string): string | null {
  if (!identifier) {
    return null
  }

  // 只允许字母、数字、下划线
  const sanitized = identifier.replaceAll(/\W/g, '')

  // 确保不是空字符串且以字母开头
  if (!sanitized || !/^[a-z]/i.test(sanitized)) {
    return null
  }

  return sanitized
}

/**
 * 验证排序方向
 *
 * @param direction - 排序方向
 * @returns 'ASC' | 'DESC'
 */
export function sanitizeOrderDirection(direction: string): 'ASC' | 'DESC' {
  const upper = (direction || '').toUpperCase()
  return upper === 'ASC' ? 'ASC' : 'DESC'
}

/**
 * 构建安全的排序子句
 *
 * @param orderBy - 排序字段
 * @param direction - 排序方向
 * @param allowedFields - 允许的字段列表
 * @param defaultField - 默认排序字段
 * @returns 排序配置对象
 */
export function buildOrderClause(
  orderBy: string | undefined,
  direction: string | undefined,
  allowedFields: string[],
  defaultField = 'createdAt',
): Record<string, 'ASC' | 'DESC'> {
  const sanitizedField = sanitizeIdentifier(orderBy || '')
  const sanitizedDirection = sanitizeOrderDirection(direction || 'DESC')

  // 只使用白名单中的字段
  const field =
    sanitizedField && allowedFields.includes(sanitizedField) ? sanitizedField : defaultField

  return { [field]: sanitizedDirection }
}
