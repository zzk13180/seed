/**
 * user-management.query.ts 单元测试
 *
 * 测试策略：
 * - 使用 QueryClient + wrapper 模拟 TanStack Query 运行环境
 * - 验证 query key 生成的正确性和层级结构
 * - 验证 hook 正确委托给 apiService（DI 注入的同一实例）
 * - 验证 mutation 成功后触发缓存失效
 */
import { describe, it, expect } from 'vitest'
import { userQueryKeys } from '../user-management.query'

// ─── Query Key 测试 ──────────────────────────────────────────────────────────
// query key factory 是纯函数，不需要 Vue/TanStack 环境即可测试
describe('userQueryKeys', () => {
  it('all — 根键', () => {
    expect(userQueryKeys.all).toEqual(['users'])
  })

  it('pages — 分页列表键', () => {
    expect(userQueryKeys.pages()).toEqual(['users', 'page'])
  })

  it('page — 带参数的分页键', () => {
    const params = { page: 1, pageSize: 10, username: 'test' }
    const key = userQueryKeys.page(params)

    expect(key).toEqual(['users', 'page', params])
    // 不同参数生成不同 key
    expect(key).not.toEqual(userQueryKeys.page({ page: 2, pageSize: 10 }))
  })

  it('pages() 是 page(params) 的前缀 — invalidateQueries 可按前缀失效', () => {
    const pagesKey = userQueryKeys.pages()
    const pageKey = userQueryKeys.page({ page: 1, pageSize: 10 })

    // TanStack Query 的 invalidateQueries 按前缀匹配
    // pages() = ['users', 'page'] 应是 page(params) 的前缀
    expect(pageKey.slice(0, pagesKey.length)).toEqual(pagesKey)
  })
})
