# Copilot Instructions — Seed Monorepo

> 本文件在每次 Copilot Chat 对话时自动注入。详细规范见对应源文件。

---

## Nx 工作区指引

- 探索工作区时，先调用 `nx-workspace` skill
- 运行任务始终通过 Nx：`pnpm nx run <project>:<target>`，不直接使用底层工具
- 脚手架/生成操作先调用 `nx-generate` skill
- 不确定 CLI 参数时查 `nx_docs` 或 `--help`，**不要猜**
- 插件最佳实践查 `node_modules/@nx/<plugin>/PLUGIN.md`（不一定存在）

---

## 技术栈

| 层级           | 技术                                           |
| -------------- | ---------------------------------------------- |
| 编排           | Nx 22 + pnpm 10（TS + Rust + Python monorepo） |
| Serverless API | Hono → Cloudflare Workers                      |
| 重型任务       | Hono on Bun (Docker)：WebSocket / Cron / 文件  |
| 认证           | Better Auth（Bearer Token，全端统一）          |
| 数据库         | Neon PostgreSQL + Drizzle ORM                  |
| 验证           | Zod（SSOT：API 契约 + DB schema + 表单）       |
| Web SPA        | Vue 3.5 + Vite 7 + Element Plus / Ionic 8      |
| 原生端         | Tauri 2 (Rust)（桌面 + Android + iOS）         |
| 文档           | Astro 5 (SSG)                                  |
| UI 组件库      | Svelte 5 Web Components                        |
| 测试           | Vitest 3.2                                     |

---

## 包依赖规则（单向）

```
Layer 3 (apps) → Layer 2 (services) → Layer 1 (kit/db) → Layer 0 (contracts/types)
```

- `apps/**` 禁止互相依赖（包括 spa/ 和 native/ 下的子项目）
- `packages/kit` 不依赖 `packages/db`
- `packages/contracts` 是纯 Layer 0，只依赖 zod

---

## 命名约定

| 类型       | 格式                    | 示例                 |
| ---------- | ----------------------- | -------------------- |
| 模块文件   | `kebab-case.{role}.ts`  | `user.controller.ts` |
| Vue 组件   | `PascalCase.vue`        | `UserManagement.vue` |
| 类         | PascalCase              | `UserService`        |
| 函数/方法  | camelCase               | `findPage()`         |
| 常量       | UPPER_SNAKE_CASE        | `RESPONSE_CODE`      |
| Zod Schema | camelCase + Schema 后缀 | `userCreateSchema`   |

---

## 后端模块约定

### 三层架构

```
packages/services/src/{name}/
├── {name}.controller.ts   # 路由 + 参数验证，只编排不含业务逻辑
├── {name}.service.ts       # 业务逻辑，抛 BusinessError，返回 VO
├── {name}.repository.ts    # Drizzle ORM 数据访问
├── {name}.schema.ts        # 模块专用 Zod schemas
└── {name}.vo.ts            # Entity → VO 转换
```

- Controller 导出工厂函数 `createXxxRoutes(db: AnyDatabase)`
- Service 抛 `NotFoundError` / `ConflictError` 等语义化错误，不手写 HTTP 状态码
- Repository 排序用白名单 `SORTABLE_FIELDS`，LIKE 用 `escapeLikeString()`

### 注册步骤

1. `packages/services/src/index.ts` 添加 re-export
2. `packages/services/package.json` 的 `exports` 添加子路径
3. `apps/api/edge/src/app.ts` 注册路由
4. `apps/api/bun/src/app.ts` 按需注册

---

## 前端模块约定（五件套）

```
views/{module}/
├── {Module}.vue                  # UI 渲染，只依赖 Store
├── {module}.types.ts             # State + Deps 接口
├── {module}.controller.ts        # 纯 TS 逻辑（无 Vue/Pinia 导入）
├── {module}.service.ts           # HTTP 调用（Hono RPC）
└── {module}.store.ts             # Pinia 胶水层（reactive + markRaw + DI）
```

- 需异步初始化/销毁 → 继承 `BaseController` from `@seed/kit/frontend`
- 纯同步 CRUD → 普通类即可
- 复杂交互追加 `{module}.query.ts`（TanStack Query，六件套）

---

## 错误处理

```
BusinessError (base)
├── ValidationError     (422)
├── UnauthorizedError   (401)
├── ForbiddenError      (403)
├── NotFoundError       (404)
└── ConflictError       (409)
```

使用语义化工厂方法：`throw new NotFoundError('User', id)`

---

## Zod Schema 位置

| 类型       | 位置                                            |
| ---------- | ----------------------------------------------- |
| 跨模块共享 | `packages/contracts/src/schemas/`               |
| 模块专用   | `packages/services/src/{name}/{name}.schema.ts` |

Schema 作为 SSOT，类型从 schema 推导：`type XxxDto = z.infer<typeof xxxSchema>`

---

## 导入优先级

1. Node.js 内置模块 (`node:path`)
2. 第三方依赖 (`hono`, `zod`)
3. Workspace 共享包 (`@seed/contracts`, `@seed/kit`, `@seed/db`)
4. 项目内部模块 (`./xxx.service`)

优先子路径导入以获得更好的 tree-shaking。

---

## 中间件链顺序

```
TraceId → SecureHeaders → CORS → RateLimiter → Logger → Transform → Session → [Route] → ErrorHandler
```

---

## 数据库表约定

- 主键：`text('id').primaryKey()`（UUID）
- 时间字段：`timestamp(..., { mode: 'date' })`
- 软删除：`deleted: integer('deleted').notNull().default(0)`

---

## Git 提交

```
<type>(<scope>): <subject>
```

type: `feat` | `fix` | `refactor` | `chore` | `docs` | `test` | `style` | `perf` | `ci`

---

## 详细参考

详细文档位于 `apps/docs/src/content/docs/`，以下为对应路径：

- 架构全景 → `apps/docs/src/content/docs/architecture/index.mdx`
- 后端架构 → `apps/docs/src/content/docs/architecture/backend.mdx`
- 前端架构 → `apps/docs/src/content/docs/architecture/frontend.mdx`
- 代码约定 → `apps/docs/src/content/docs/conventions.mdx`
- 脚手架模板 → `apps/docs/src/content/docs/templates.mdx`
