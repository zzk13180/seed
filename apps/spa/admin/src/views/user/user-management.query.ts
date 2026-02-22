/**
 * UserManagement 模块 — TanStack Query 层（六件套第六件）
 *
 * 职责：为用户管理模块提供 **声明式数据获取** 能力。
 * 相比五件套的命令式 `controller.fetchUserList()`，
 * 六件套增加了自动缓存、后台刷新、窗口聚焦重取等 TanStack Query 特性。
 *
 * 架构约束：
 * - query hooks 通过 apiService（DI 注入的同一实例）获取数据
 * - 不绕过 Controller 直接构造请求
 * - mutation hooks 成功后自动失效相关缓存
 *
 * 数据流：
 *   Vue 组件 → useUserPageQuery(apiService, params)
 *                 └──▶ apiService.getPage()  ← 同一 DI 实例
 *                        └──▶ Hono RPC client
 *
 * @module views/user/user-management.query
 */

import { computed, type MaybeRefOrGetter, toValue } from 'vue'
import { useQuery, useMutation, useQueryClient } from '@tanstack/vue-query'
import type {
  UserManagementApiService,
  UserQueryDto,
  UserCreateDto,
  UserUpdateDto,
} from './user-management.types'
import type { UserStatus } from '@seed/contracts'

// ============================================================================
// Query Key Factory — 集中管理缓存键，确保失效操作精确
// ============================================================================

export const userQueryKeys = {
  /** 所有用户相关查询的根键 */
  all: ['users'] as const,
  /** 分页列表查询 */
  pages: () => [...userQueryKeys.all, 'page'] as const,
  /** 带参数的分页查询 */
  page: (params: Partial<UserQueryDto>) => [...userQueryKeys.pages(), params] as const,
}

// ============================================================================
// Query Hooks — 声明式数据获取
// ============================================================================

/**
 * useUserPageQuery — 用户分页列表查询
 *
 * 自动缓存 + 后台刷新 + 窗口聚焦重取。
 * 适用于需要轮询或实时性较高的场景。
 *
 * @param apiService - DI 注入的用户 API 服务（与 Controller 使用同一实例）
 * @param params - 响应式查询参数（分页 + 搜索条件）
 * @param options - TanStack Query 选项（refetchInterval 启用轮询）
 *
 * @example
 * ```typescript
 * const { data, isLoading, error } = useUserPageQuery(
 *   apiService,
 *   () => ({ page: 1, pageSize: 10 }),
 *   { refetchInterval: 30_000 }, // 每 30 秒自动刷新
 * )
 * ```
 */
export function useUserPageQuery(
  apiService: UserManagementApiService,
  params: MaybeRefOrGetter<UserQueryDto>,
  options?: {
    /** 自动刷新间隔（毫秒），适用于轮询场景 */
    refetchInterval?: number
    /** 是否启用查询（响应式） */
    enabled?: MaybeRefOrGetter<boolean>
  },
) {
  return useQuery({
    queryKey: computed(() => userQueryKeys.page(toValue(params))),
    queryFn: () => apiService.getPage(toValue(params)),
    refetchInterval: options?.refetchInterval,
    enabled: options?.enabled,
  })
}

// ============================================================================
// Mutation Hooks — 声明式数据变更（自动失效缓存）
// ============================================================================

/**
 * useCreateUserMutation — 创建用户
 *
 * 成功后自动失效分页列表缓存，触发 UI 重新获取最新数据。
 */
export function useCreateUserMutation(apiService: UserManagementApiService) {
  const queryClient = useQueryClient()
  return useMutation({
    mutationFn: (params: UserCreateDto) => apiService.create(params),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: userQueryKeys.pages() })
    },
  })
}

/**
 * useUpdateUserMutation — 更新用户
 *
 * 成功后自动失效分页列表缓存。
 */
export function useUpdateUserMutation(apiService: UserManagementApiService) {
  const queryClient = useQueryClient()
  return useMutation({
    mutationFn: ({ id, params }: { id: string; params: UserUpdateDto }) =>
      apiService.update(id, params),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: userQueryKeys.pages() })
    },
  })
}

/**
 * useDeleteUserMutation — 删除用户
 *
 * 成功后自动失效分页列表缓存。
 */
export function useDeleteUserMutation(apiService: UserManagementApiService) {
  const queryClient = useQueryClient()
  return useMutation({
    mutationFn: (id: string) => apiService.delete(id),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: userQueryKeys.pages() })
    },
  })
}

/**
 * useBatchDeleteUserMutation — 批量删除用户
 *
 * 成功后自动失效分页列表缓存。
 */
export function useBatchDeleteUserMutation(apiService: UserManagementApiService) {
  const queryClient = useQueryClient()
  return useMutation({
    mutationFn: (ids: string[]) => apiService.deleteBatch(ids),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: userQueryKeys.pages() })
    },
  })
}

/**
 * useUpdateUserStatusMutation — 更新用户状态
 *
 * 成功后自动失效分页列表缓存。
 */
export function useUpdateUserStatusMutation(apiService: UserManagementApiService) {
  const queryClient = useQueryClient()
  return useMutation({
    mutationFn: ({ id, status }: { id: string; status: UserStatus }) =>
      apiService.updateStatus(id, status),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: userQueryKeys.pages() })
    },
  })
}

/**
 * useResetPasswordMutation — 重置用户密码
 */
export function useResetPasswordMutation(apiService: UserManagementApiService) {
  const queryClient = useQueryClient()
  return useMutation({
    mutationFn: ({ id, password }: { id: string; password: string }) =>
      apiService.resetPassword(id, password),
    onSuccess: () => {
      void queryClient.invalidateQueries({ queryKey: userQueryKeys.pages() })
    },
  })
}
