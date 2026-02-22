# @seed/services — 共享业务模块

> **架构层级**: Layer 2 — Domain Modules（可复用的业务逻辑单元）

## AI 参考指引

- **新增后端 API 必须在此包创建模块**，不要写在 `apps/api/` 下
- 模块遵循三层架构：controller（路由）→ service（业务）→ repository（数据访问）
- Controller 导出工厂函数 `createXxxRoutes(db)`，接收 db 实例作为参数
- 新增模块后需在 `src/index.ts` re-export，`package.json` exports 添加子路径，app 的 `app.ts` 注册路由
- 错误抛出语义化 BusinessError（NotFoundError / ConflictError 等），不手写 HTTP 状态码

## 定位

业务代码只写一份，`apps/api/edge/` 和 `apps/api/bun/` 都从这里导入。每个模块导出工厂函数，接收数据库实例作为参数（依赖注入），app 层只负责组装。

## 目录结构

```
src/
├── auth/                  # 认证模块（Better Auth 路由代理）
│   ├── auth.controller.ts # createAuthRoutes(auth) → Hono
│   └── index.ts
├── health/                # 健康检查模块
│   ├── health.controller.ts  # createHealthRoutes(db) → Hono
│   ├── health.service.ts
│   └── index.ts
├── user/                  # 用户管理模块（标准 CRUD）
│   ├── user.controller.ts # createUserRoutes(db) → Hono
│   ├── user.repository.ts # 数据访问层
│   ├── user.service.ts    # 业务逻辑层
│   ├── user.vo.ts         # 视图对象转换
│   ├── __tests__/         # 单元测试
│   └── index.ts
├── tauri-updater/         # Tauri 桌面端更新模块
│   ├── tauri-updater.controller.ts  # createTauriUpdaterRoutes() → Hono
│   ├── tauri-updater.service.ts
│   └── index.ts
├── api-preset.ts          # registerCoreRoutes() — 核心路由批量注册
└── index.ts               # Barrel export

```

## 使用方式

```typescript
// apps/api/edge/src/app.ts
import {
  createAuthRoutes,
  createHealthRoutes,
  createUserRoutes,
  createTauriUpdaterRoutes,
} from '@seed/services'

const database = db()
app.route('/api/auth', createAuthRoutes(auth))
app.route('/api/health', createHealthRoutes(database))
app.route('/api/v1/users', createUserRoutes(database))
app.route('/api/updater', createTauriUpdaterRoutes())
```

## 模块工厂函数签名

| 模块          | 工厂函数                     | 依赖             |
| ------------- | ---------------------------- | ---------------- |
| auth          | `createAuthRoutes(auth)`     | Better Auth 实例 |
| health        | `createHealthRoutes(db)`     | Database         |
| user          | `createUserRoutes(db)`       | Database         |
| tauri-updater | `createTauriUpdaterRoutes()` | 无               |

## 新增模块步骤

1. 创建 `src/{name}/` 目录
2. 编写 `{name}.controller.ts` — 导出 `createXxxRoutes(db)` 工厂函数
3. 编写 `{name}.service.ts` — 业务逻辑
4. 如需数据访问，编写 `{name}.repository.ts`
5. 如需请求验证，编写 `{name}.schema.ts`（Zod）
6. 编写 `index.ts` 导出公开 API
7. 在 `src/index.ts` 添加 re-export
8. 在 `package.json` 的 `exports` 添加子路径
9. 在 app 的 `app.ts` 中 `app.route()` 注册

## 依赖规则

```
@seed/services → @seed/contracts (schemas, errors, enums)
@seed/services → @seed/kit (hono middleware, utils)
@seed/services → @seed/db (schema types)
@seed/services → hono, zod, drizzle-orm (peer-level)
@seed/services ✗ apps/* (禁止反向依赖)
```
