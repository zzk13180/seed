/**
 * @seed/api-types - 共享 API 类型定义
 *
 * 此包定义了前后端共享的纯类型（interface/type/enum），
 * 不包含任何运行时代码（class/function）。
 *
 * 使用方式：
 * - 前端：直接使用类型定义
 * - 后端：DTO/VO class 实现这些接口
 */

// 枚举
export * from './enums/index'

// 通用类型
export * from './common/index'

// 认证模块
export * from './auth/index'

// 用户模块
export * from './user/index'
