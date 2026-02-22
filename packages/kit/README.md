# @seed/kit

> 跨应用共享基础设施库 (Layer 1) — Auth 配置工厂 · Hono 中间件 · 前端 BaseController · 工具函数

## AI 参考指引

- **所有后端项目都应依赖此包**：它提供 Hono 中间件链、Better Auth 配置工厂等运行时基础设施
- **纯类型/schemas/enums/errors 已迁移到 `@seed/contracts` (Layer 0)**
- 新增的 Hono 中间件放在 `hono/middleware/` 下，不要在 app 内创建中间件
- `auth/` 是 Better Auth 的配置工厂，server 和 client 端分别导入
- `hono` 和 `better-auth` 都是可选 peer dep，非 Hono/非 Auth 消费者不需要安装

## 导出入口（exports map）

| 路径 | 内容 | 使用方 |
| --- | --- | --- |
| `@seed/kit` | 主入口（minimal） | — |
| `@seed/kit/utils` | 工具函数（date-fns wrappers, escapeLikeString 等） | 全部 |
| `@seed/kit/frontend` | BaseController + Logger（纯 TS） | spa/admin, spa/console |
| `@seed/kit/auth/server` | Better Auth 服务端配置工厂 | api/edge, api/bun |
| `@seed/kit/hono/middleware` | Hono 中间件集合 | api/edge, api/bun, services |
| `@seed/kit/hono/app-factory` | createHonoApp() 工厂 | api/edge, api/bun |

> **注意**：Schemas / Enums / Errors 已迁移至 `@seed/contracts`，请使用 `@seed/contracts/schemas/*`、`@seed/contracts/errors` 等路径

## 目录结构

```
src/
├── index.ts                # 主入口（minimal export）
├── auth/
│   └── server.ts           # Better Auth 服务端配置工厂
├── frontend/
│   ├── base.controller.ts  # BaseController 抽象类
│   └── logger.service.ts   # 日志服务
├── hono/
│   ├── app-factory.ts      # createHonoApp() 中间件链组装工厂
│   └── middleware/
│       ├── index.ts         # barrel export
│       ├── auth-guard.middleware.ts
│       ├── error-handler.middleware.ts
│       ├── logger.middleware.ts
│       ├── trace-id.middleware.ts
│       └── transform.middleware.ts
└── utils/
    ├── index.ts
    └── *.util.ts            # date, base64, http-status, sql (escapeLikeString)
```

## 依赖关系

```
@seed/kit → @seed/contracts（errors, enums — 中间件需要 ResponseCode 等）
           → zod（error-handler 需要捕获 ZodError）

@seed/kit ← @seed/services
           ← @seed/api-edge
           ← @seed/api-bun
           ← @seed/spa-admin
           ← @seed/spa-console

可选 peer dependencies:
  - hono（仅 hono/* 导出需要）
  - better-auth（仅 auth/* 导出需要）
  - hono-rate-limiter（仅 hono/app-factory 需要）
```

## 开发命令

```bash
pnpm nx type-check @seed/kit   # 类型检查
```

## 添加新内容的规范

- **新中间件**: 在 `hono/middleware/` 下创建 `xxx.middleware.ts`，在 `hono/middleware/index.ts` re-export
- **新工具函数**: 在 `utils/` 下创建 `xxx.util.ts`，在 `utils/index.ts` re-export
- **新 schema/enum/error**: 放在 `@seed/contracts` 而非此包
