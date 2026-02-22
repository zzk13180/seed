# @seed/api-edge

> Hono 轻量 API 服务 — Cloudflare Workers 边缘部署（处理 80% 无状态请求）

## AI 参考指引

- **新建 Serverless API 时优先参考此项目**：Hono + Neon HTTP 无状态架构
- 模块结构与 `apps/api/bun/` 完全一致（Controller → Service → Repository），但本项目使用 Neon HTTP 驱动（无状态）
- 中间件通过 `@seed/kit/hono/middleware` 共享，**不要**在 app 内新建中间件
- `worker.entry.ts` 为 Cloudflare Workers 的入口，`main.ts` 为 Bun 直接运行的入口
- 认证统一使用 Bearer Token 模式

## 技术栈

| 维度   | 技术                                  |
| ------ | ------------------------------------- |
| 运行时 | Bun / Cloudflare Workers              |
| 框架   | Hono                                  |
| 认证   | Better Auth（Bearer Token）           |
| ORM    | Drizzle ORM（共享 `@seed/db`）        |
| 数据库 | Neon PostgreSQL HTTP（无状态连接）    |
| 验证   | Zod（共享 `@seed/contracts/schemas`） |
| 限流   | hono-rate-limiter                     |
| 部署   | Cloudflare Workers（wrangler）        |

## 目录结构

```
src/
├── app.ts                  # Hono 应用工厂（registerCoreRoutes 注册共享路由）
├── main.ts                 # Bun 入口
├── worker.entry.ts         # Cloudflare Workers 入口
├── config/                 # 环境变量（Zod 验证）+ Better Auth 实例
└── database/               # Neon HTTP 连接 + Drizzle 实例
```

## Nx 任务

```bash
pnpm nx run @seed/api-edge:dev       # 热重载开发（bun --hot）
pnpm nx run @seed/api-edge:build     # 构建
pnpm nx run @seed/api-edge:test      # 测试
pnpm nx run @seed/api-edge:deploy    # Cloudflare Workers 部署
```

## 与 api/bun 的关系

| 维度   | api/edge（本项目）            | api/bun           |
| ------ | ----------------------------- | ----------------- |
| 定位   | 80% 无状态请求                | 20% 有状态/长连接 |
| 数据库 | Neon HTTP（无连接池）         | pg TCP（连接池）  |
| 部署   | Cloudflare Workers            | Docker            |
| 入口   | `worker.entry.ts` / `main.ts` | `main.ts`         |
| 共享   | @seed/kit, @seed/db, 中间件   | 同左              |

## 集合目录说明

`apps/api/` 是后端 API 服务的集合目录：

```
apps/api/
├── edge/       # 边缘计算 (Cloudflare Workers, Neon HTTP)
├── bun/        # Bun Docker (pg TCP, WebSocket/Cron/文件)
└── {new-api}/  # 新增的 API 部署目标
```
