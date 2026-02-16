# ARCHITECTURE.md — Seed Monorepo 架构全景

> 本文档是 **AI Agent 和人类开发者** 的共同参考标准。修改架构时，此文档必须同步更新。

---

## 技术栈总览

| 层级 | 技术选型 | 说明 |
| --- | --- | --- |
| **任务编排** | Nx 22.x + pnpm 10 | polyglot monorepo（TS + Rust + Python） |
| **Serverless API** | Hono (14KB) | Cloudflare Workers / Bun 双运行时 |
| **重型任务** | Hono on Bun (Docker) | WebSocket / Cron / 文件处理 |
| **认证** | Better Auth | Session-based，Cookie + Bearer 双模式 |
| **数据库** | Neon PostgreSQL + Drizzle ORM | HTTP driver (Workers) / TCP driver (Docker) |
| **验证** | Zod | SSOT：API 契约 + DB schema + 表单校验 |
| **前端** | Vue 3.5 + Vite 7 + Element Plus | Controller + Store 分层架构 |
| **移动端** | Vue 3.5 + Ionic 8 | 共享 @seed/kit schemas |
| **桌面端** | Tauri 2 (Rust + WebView) | 跨平台桌面应用 |
| **文档** | Astro 5 (SSG) | 静态站点 |
| **UI 组件库** | Svelte 5 Web Components | 框架无关 |
| **Lint** | ESLint flat config | 共享 @seed/eslint-config |
| **测试** | Vitest 3.2 | 单元 + 集成测试 |

---

## 工作区结构

```
seed-monorepo/
├── Cargo.toml            # Cargo workspace (Rust 共享 crates + Tauri apps)
├── crates/
│   └── seed-common/      # 共享 Rust 工具 crate (platform detection 等)
│
├── apps/
│   ├── api/              # Hono 后端 (Serverless → CF Workers, 80% 流量)
│   ├── server/           # Hono on Bun (Docker, 20% 流量: WS/Cron/文件)
│   ├── admin/            # Vue 3 管理后台
│   ├── mobile/           # Vue 3 + Ionic 移动端
│   ├── desktop/          # Tauri 2 桌面端
│   ├── docs/             # Astro 文档站
│   └── robotics/         # ROS2 Python (排除于 pnpm workspace)
│
├── packages/
│   ├── contracts/        # 契约层 (@seed/contracts) — Layer 0
│   │   ├── schemas/      # Zod schemas (SSOT)
│   │   ├── enums/        # 共享枚举
│   │   └── errors/       # 统一错误体系
│   │
│   ├── kit/              # 基础设施层 (@seed/kit) — Layer 1
│   │   ├── utils/        # 纯工具函数
│   │   ├── auth/         # Better Auth 配置工厂 (createAuth)
│   │   ├── hono/         # Hono 中间件 + createHonoApp() 工厂
│   │   │   ├── middleware/ # traceId · logger · transform · errorHandler · authGuard
│   │   │   └── app-factory # 中间件链组装工厂，app.ts ~20 行即可完成
│   │   └── frontend/     # BaseController + Logger（纯 TS，跨前端 app 共享）
│   │
│   ├── db/               # 数据库层 (@seed/db)
│   │   ├── schema/       # Drizzle schema (Better Auth 表)
│   │   ├── types.ts      # AnyDatabase 通用类型（兼容 Neon HTTP + pg TCP）
│   │   └── provider.ts   # Neon HTTP 连接工厂
│   │
│   ├── services/         # 共享业务模块 (@seed/services)
│   │   ├── auth/         # Better Auth 代理路由
│   │   ├── health/       # 存活/就绪检查
│   │   ├── user/         # 用户 CRUD (ctrl + svc + repo + schema + vo)
│   │   └── tauri-updater/# Tauri 自动更新 (server-only, 依赖 node:fs)
│   │
│   ├── ui/               # Svelte 5 Web Components (@seed/ui)
│   │
│   ├── configs/          # 工具链配置
│   │   ├── eslint/       # @seed/eslint-config
│   │   ├── tsconfig/     # @seed/tsconfig
│   │   ├── tailwind/     # @seed/tailwind-config
│   │   └── vite/         # @seed/vite-config
│   │
│   └── libs/
│       └── types/        # 全局 TS 类型声明
│
├── TODO.md               # 架构进度跟踪 + 待办
├── ARCHITECTURE.md       # ← 本文件
├── CONVENTIONS.md        # 代码约定
└── TEMPLATES.md          # AI 脚手架手册（7 种操作模板）
```

---

## 包依赖规则（单向，由 `@nx/enforce-module-boundaries` 强制执行）

```
Layer 3 (apps)      →  Layer 2 (services)  →  Layer 1 (kit / db)  →  Layer 0 (contracts / types)
apps/*              →  packages/services   →  packages/kit + packages/db + packages/contracts
apps/* ✗ apps/*                              (禁止应用间直接依赖)
packages/services    →  packages/kit + packages/db + packages/contracts
packages/kit       →  packages/contracts  (middleware 引用 errors/schemas)
packages/kit ✗ packages/db                  (middleware 不依赖 drizzle 表定义)
packages/contracts  ✗ 任何其他 package       (纯 Layer 0，只依赖 zod)
packages/db         →  drizzle-orm (@neondatabase/serverless 为 optional peer dep)
```

---

## 请求生命周期

```
Client → CDN → Cloudflare Worker → Hono Middleware Chain → Controller → Service → Repository → Drizzle → Neon
```

### 中间件链顺序

```
TraceId → SecureHeaders → CORS → RateLimiter → Logger → Transform → Session → [Route Handler] → ErrorHandler
```

| 中间件          | 职责                                                             |
| --------------- | ---------------------------------------------------------------- |
| `traceId`       | 生成/传递请求追踪 ID                                             |
| `secureHeaders` | 安全 HTTP 响应头                                                 |
| `cors`          | 跨域配置                                                         |
| `rateLimiter`   | 速率限制                                                         |
| `logger`        | 请求/响应日志（敏感字段脱敏）                                    |
| `transform`     | 统一包装成功响应为 `{ code, data, message, timestamp, traceId }` |
| `session`       | Better Auth session 注入到 Hono Context                          |
| `errorHandler`  | 全局异常处理 (`BusinessError` / `ZodError` / `HTTPException`)    |

---

## 后端模块架构 (Controller → Service → Repository)

每个业务模块由 3 层组成，职责边界清晰：

```
packages/services/src/{name}/
├── {name}.controller.ts   # 路由定义 + 参数验证 (Zod) + 响应构建
├── {name}.service.ts       # 业务逻辑 + 异常抛出 + VO 转换
├── {name}.repository.ts    # Drizzle ORM 数据访问
├── {name}.schema.ts        # 模块专用 Zod schemas (extends @seed/contracts)
└── {name}.vo.ts            # Entity → VO 转换
```

**层级规则**：

- Controller **只**调用 Service，不直接访问 Repository 或 ORM
- Service **只**调用 Repository，不处理 HTTP 细节
- Repository **只**操作 ORM，不包含业务逻辑

---

## 前端模块架构 (Controller + Store)

### 五件套（标准模式）

```
views/{module}/
├── {Module}.vue                  # 视图层 (绑定 store.state, 调用 store.controller)
├── {module}.types.ts             # State + Deps 接口 + re-export Zod schemas
├── {module}.controller.ts        # 纯 TS 业务逻辑（见下方 BaseController 约定）
├── {module}.service.ts           # Service 实现 (内部用 Hono RPC)
└── {module}.store.ts             # Pinia 胶水层 (reactive + markRaw + DI)
```

### 六件套（复杂交互追加 TanStack Query）

```
└── {module}.query.ts             # TanStack Query hooks (委托 apiService)
```

### BaseController 继承约定

| 场景 | 是否继承 BaseController | 说明 |
| --- | --- | --- |
| 需要异步初始化 / 销毁生命周期 | ✅ extends BaseController | 如 `AppController`（RxJS 订阅、WebSocket 连接） |
| 纯同步 CRUD 操作 | ❌ 普通类即可 | 如 `UserManagementController`（无异步资源需要管理） |

> BaseController 提供 `initialize()` / `dispose()` 竞态保护和重试机制。当 Controller 不需要异步生命周期时，使用普通类可以减少复杂度。

**核心原则**：

- Controller 是 **纯 TS 类**，不导入 Vue/Pinia/Hono
- Service 通过 DI 接口注入，不在 Controller 中直接实例化
- TanStack Query 是可选增强，通过 apiService（DI 同一实例）获取数据，不绕过 Service 层

---

## 认证架构 (Better Auth)

| 端              | 认证模式       | 实现                         |
| --------------- | -------------- | ---------------------------- |
| Admin (Web)     | Cookie Session | 最安全，HttpOnly + CSRF 防护 |
| Mobile (Ionic)  | Bearer Token   | 跨域场景更可靠               |
| Desktop (Tauri) | Cookie Session | Tauri WebView 支持 cookie    |

**服务端配置**：`@seed/kit/auth/server` 提供 `createAuth()` 工厂  
**客户端配置**：各前端 app 内部实现（Vue / React 特化）

---

## 双后端架构

|              | Serverless (api/)                      | Docker (server/)                      |
| ------------ | -------------------------------------- | ------------------------------------- |
| **运行时**   | Cloudflare Workers (V8)                | Bun                                   |
| **流量占比** | 80%                                    | 20%                                   |
| **DB 驱动**  | Neon HTTP (`@neondatabase/serverless`) | Neon TCP (`pg`)                       |
| **适用场景** | 无状态 CRUD, Auth, Updater             | WebSocket, Cron, 文件处理             |
| **共享**     | @seed/kit + @seed/db + @seed/services  | @seed/kit + @seed/db + @seed/services |

---

## Nx 缓存策略

```jsonc
{
  "namedInputs": {
    "serverSource": [
      "{projectRoot}/src/**/*.ts",
      "{workspaceRoot}/packages/contracts/src/**",
      "{workspaceRoot}/packages/kit/src/hono/**",
      "{workspaceRoot}/packages/kit/src/auth/**",
      "{workspaceRoot}/packages/kit/src/utils/**",
      "{workspaceRoot}/packages/db/**",
      "{workspaceRoot}/packages/services/**",
    ],
    "frontendSource": [
      "{projectRoot}/src/**/*.{ts,vue,tsx,svelte}",
      "{workspaceRoot}/packages/contracts/src/**",
      "{workspaceRoot}/packages/kit/src/frontend/**",
    ],
    "rustSource": ["{projectRoot}/**/*.rs", "{projectRoot}/Cargo.toml"],
  },
}
```

可缓存任务：`build` / `test` / `lint` / `type-check`  
不缓存任务：`dev` / `preview`

---

## 安全措施

1. **Zod 验证** — 所有 API 入参使用 Zod schema 运行时校验
2. **LIKE 转义** — `escapeLikeString()` 防止 SQL LIKE 注入
3. **排序白名单** — Repository 层的 `SORTABLE_FIELDS` 防止 ORDER BY 注入
4. **Session-based 认证** — 无 JWT，session 可服务端即时撤销
5. **Secure Headers** — CSP / X-Frame-Options / HSTS
6. **CORS 白名单** — 生产环境使用显式域名白名单，禁止反射任意来源
7. **Rate Limiting** — 按 IP 限流（CF Workers 优先使用 `cf-connecting-ip`）
8. **敏感字段脱敏** — Logger 中间件递归脱敏 password/token 等字段（深度限制 3 层）
9. **响应头保留** — Transform 中间件保留上游安全头（CSP/CORS/Set-Cookie）
10. **软删除** — 用户数据使用 `deleted` 标记，不物理删除
11. **依赖边界强制** — `@nx/enforce-module-boundaries` ESLint 规则禁止跨层违规 import
12. **版本统一** — pnpm catalogs 集中管理核心依赖版本，消除版本分裂

---

## 部署架构

```
                    ┌──────────────────────────────────────┐
                    │      Cloudflare Workers               │
                    │  apps/api/ (Hono, Serverless)         │
                    │  Auth · CRUD · Updater · Health       │
                    │           ↓                           │
                    │  Neon PostgreSQL (HTTP)                │
                    └──────────────────────────────────────┘

                    ┌──────────────────────────────────────┐
                    │      Docker Container                 │
                    │  apps/server/ (Hono on Bun)           │
                    │  WebSocket · Cron · 文件处理          │
                    │           ↓                           │
                    │  Neon PostgreSQL (TCP)                 │
                    └──────────────────────────────────────┘

                    ┌──────────────────────────────────────┐
                    │      Cloudflare Pages / Docker        │
                    │  apps/admin/  (Vue SPA)               │
                    │  apps/docs/   (Astro SSG)             │
                    └──────────────────────────────────────┘
```
