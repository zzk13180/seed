# TEMPLATES.md — AI 脚手架操作手册

> 本文件是 AI Agent 的操作手册。当需要「给项目加一个 X」时，按照对应模板执行。每个模板列出 **位置、文件清单、注册步骤、参考示例**。

---

## 新增 Hono 后端业务模块

> 频率: 高 | 执行者: AI | 期望耗时: <5 min

**位置**: `packages/services/src/{name}/`

**文件清单**:

| 文件                   | 职责                                                | 必需 |
| ---------------------- | --------------------------------------------------- | ---- |
| `{name}.controller.ts` | `createXxxRoutes(db)` 工厂函数，路由定义 + 请求验证 | ✅   |
| `{name}.service.ts`    | 业务逻辑，接收 db 或 repository                     | ✅   |
| `{name}.repository.ts` | Drizzle ORM 数据访问（有持久化需求时）              | 可选 |
| `{name}.schema.ts`     | Zod 请求验证 schemas                                | 可选 |
| `{name}.vo.ts`         | Entity → VO 转换（隐藏内部字段）                    | 可选 |
| `index.ts`             | 公开导出                                            | ✅   |

**步骤**:

1. 创建 `packages/services/src/{name}/` 目录及上述文件
2. Controller 必须导出工厂函数 `createXxxRoutes(db: AnyDatabase)`
3. 在 `packages/services/src/index.ts` 添加 re-export
4. 在 `packages/services/package.json` 的 `exports` 添加 `"./{name}"` 子路径
5. 在 `apps/api/src/app.ts` 中 `app.route('/api/v1/{name}', createXxxRoutes(database))` 注册
6. 在 `apps/server/src/app.ts` 中同样注册（如果 server 也需要）

**参考**: `packages/services/src/user/` (标准 CRUD 五件套)

**依赖注入模式**:

```typescript
import type { AnyDatabase } from '@seed/db'

export function createXxxRoutes(db: AnyDatabase) {
  const service = new XxxService(db)
  return new Hono()
    .get('/', async (c) => { ... })
}
```

---

## 新增 Vue 前端页面模块（五件套）

> 频率: 高 | 执行者: AI | 期望耗时: <10 min

**位置**: `apps/{admin|mobile}/src/views/{name}/`

**文件清单（五件套，默认复杂度）**:

| 文件                   | 职责                                                                |
| ---------------------- | ------------------------------------------------------------------- |
| `{Name}.vue`           | UI 渲染，只依赖 Store                                               |
| `{name}.types.ts`      | State + Deps 类型定义                                               |
| `{name}.controller.ts` | 纯 TS 业务逻辑（需异步初始化时 extends BaseController，否则普通类） |
| `{name}.service.ts`    | HTTP 调用（Hono RPC）                                               |
| `{name}.store.ts`      | Pinia store，桥接 Vue 响应式和 Controller                           |

**复杂度分级**:

| 级别      | 组成                                   | 适用                      |
| --------- | -------------------------------------- | ------------------------- |
| 简单 CRUD | vue + types + service + store (四件套) | 字典管理                  |
| 标准业务  | 四件套 + controller (五件套，默认)     | 用户管理                  |
| 复杂交互  | 五件套 + query.ts (六件套)             | 地图模块（轮询/乐观更新） |

**步骤**:

1. 创建 `views/{name}/` 目录及文件
2. `{name}.types.ts` 定义 `State` 和 `Deps` 接口
3. `{name}.controller.ts`:
   - 需要异步初始化/销毁（如 WebSocket、RxJS 订阅）→ 继承 `BaseController<State, Deps>` from `@seed/kit/frontend`
   - 纯同步 CRUD 逻辑 → 普通类，接收 `(state, deps)` 构造参数即可
4. `{name}.store.ts` 创建 Pinia store，使用 `reactive(state)` + `markRaw(controller)`
5. `{Name}.vue` 从 store 获取 state 并渲染
6. 添加路由（`router/index.ts`）
7. **（六件套额外步骤）** `{name}.query.ts` 定义 TanStack Query hooks，通过 apiService 获取数据

**参考**: `apps/admin/src/views/user/`（五件套 CRUD） / `user-management.query.ts`（六件套示例）

---

## 新增 Zod Schema（跨前后端共享）

> 频率: 中 | 执行者: AI | 期望耗时: <3 min

**位置**: `packages/contracts/src/schemas/{name}.schema.ts`

**步骤**:

1. 创建 schema 文件，使用 `z.object({...})`
2. 在 `packages/contracts/src/schemas/index.ts` 添加 re-export
3. （可选）在 `packages/contracts/package.json` 添加 `"./schemas/{name}"` 子路径导出

**参考**: `packages/contracts/src/schemas/user.schema.ts`

**约定**:

- Schema 名以 `Schema` 后缀: `userQuerySchema`, `loginSchema`
- 使用 `z.infer<typeof xxxSchema>` 导出类型
- 复用 `pageRequestSchema` 做分页基础

---

## 新增 Drizzle 数据库表

> 频率: 低 | 执行者: AI+人 | 期望耗时: <10 min

**位置**: `packages/db/src/schema/{name}.schema.ts`

**步骤**:

1. 创建表定义文件，使用 `pgTable(...)`
2. 在 `packages/db/src/schema/index.ts` 添加 re-export
3. 在 `packages/db/src/index.ts` 添加实体类型导出
4. 运行数据库迁移:
   ```bash
   pnpm nx db:generate api
   pnpm nx db:migrate api
   ```

**参考**: `packages/db/src/schema/auth.schema.ts`

**约定**:

- 主键使用 `text('id').primaryKey()`（UUID）
- 时间字段: `createdAt`, `updatedAt` 使用 `timestamp(..., { mode: 'date' })`
- 软删除: `deleted: integer('deleted').notNull().default(0)`

---

## 新增 Hono 中间件

> 频率: 低 | 执行者: AI | 期望耗时: <5 min

**位置**: `packages/kit/src/hono/middleware/{name}.middleware.ts`

**步骤**:

1. 创建中间件文件
2. 在 `packages/kit/src/hono/middleware/index.ts` 添加 re-export
3. 在 app 的 `app.ts` 中注册: `app.use('*', xxxMiddleware)` 或 `app.use('/path', xxxMiddleware)`

**中间件链顺序**: TraceId → SecureHeaders → CORS → RateLimiter → Logger → Transform → Session → [Route] → ErrorHandler

**参考**: `packages/kit/src/hono/middleware/trace-id.middleware.ts`

---

## 新增前端应用

> 频率: 低 | 执行者: AI+人 | 期望耗时: <30 min

**步骤**:

1. 创建 `apps/{name}/` 目录
2. 初始化 `package.json`（依赖 `@seed/kit`, `@seed/types` 等）
3. 创建 `project.json`:
   ```json
   {
     "tags": ["type:app", "platform:xxx", "framework:xxx"]
   }
   ```
4. 创建 `tsconfig.json`（继承 `@seed/tsconfig`）
5. 创建标准化 `README.md`（AI 参考指引格式）
6. 前端应用必须依赖 `@seed/kit/frontend` 获取 BaseController + Logger
7. 依赖 `@seed/contracts/schemas` 获取共享类型

**README 格式**:

```markdown
# 项目名

> 一句话描述

## AI 参考指引

- 技术栈: XXX
- 入口: src/main.ts
- 路由: src/router/
- ...

## 技术栈

## 目录结构

## 依赖关系

## 开发命令
```

**参考**: 任何现有 app 的 README.md

---

## 新增共享工具函数

> 频率: 低 | 执行者: AI | 期望耗时: <3 min

**位置**: `packages/kit/src/utils/{name}.util.ts`

**步骤**:

1. 创建工具函数文件（纯函数，零副作用）
2. 在 `packages/kit/src/utils/index.ts` 添加 re-export
3. 编写 JSDoc 注释 + @example

**约定**: 工具函数必须是纯函数，无框架依赖，可 tree-shake

---

## 项目自检清单

执行任何模板后，确认以下四项：

1. ✅ 新增后端模块是否只写了一份代码？（在 `packages/services/`）
2. ✅ 新增前端 Controller 是否继承了 `BaseController`？（from `@seed/kit/frontend`）
3. ✅ 共享包的 import 是否会拖入不需要的依赖？
4. ✅ 文档是否更新？（README.md / TEMPLATES.md / TODO.md）
