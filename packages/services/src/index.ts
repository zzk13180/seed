/**
 * @seed/services — 共享业务模块
 *
 * 业务代码只写一份，apps/api 和 apps/server 都从这里导入。
 * 每个模块导出工厂函数，接收依赖（db, auth）作为参数。
 */

// API 路由预设 — 封装公共路由注册 + 导出 RPC 类型
export { registerCoreRoutes } from './api-preset'
export type { CoreRoutesApp, CoreRouteDeps, AuthHandler } from './api-preset'

// Auth — Better Auth 路由代理
export { createAuthRoutes } from './auth/index'

// Health — 健康检查（liveness + readiness）
export { createHealthRoutes } from './health/index'

// User — 用户管理 CRUD
export { createUserRoutes } from './user/index'
export type {
  UserVO,
  UserQuery,
  UserUpdateDto,
  UserCreateDto,
  UserRoutesOptions,
  HashPasswordFn,
} from './user/index'

// Tauri Updater — 桌面端版本更新
export { createTauriUpdaterRoutes } from './tauri-updater/index'
export type { UpdateResponse } from './tauri-updater/index'
