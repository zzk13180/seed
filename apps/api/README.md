# @seed/api

> Hono + Bun 轻量 API 服务（Cloudflare Workers / Bun 双部署，处理 80% 无状态请求）

## AI 参考指引

- **新建 Hono API 项目时优先参考此项目**：它是最标准的 Hono on Bun Serverless 架构
- 模块结构与 `apps/server/` 完全一致（Controller → Service → Repository），但本项目使用 Neon HTTP 驱动（无状态）
- 中间件通过 `@seed/kit/hono/middleware` 共享，**不要**在 app 内新建中间件
- `worker.entry.ts` 为 Cloudflare Workers 的入口，`main.ts` 为 Bun 直接运行的入口

## 技术栈

| 维度   | 技术                                  |
| ------ | ------------------------------------- |
| 运行时 | Bun / Cloudflare Workers              |
| 框架   | Hono                                  |
| 认证   | Better Auth（共享 `@seed/kit/auth`）  |
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

## 中间件链（顺序）

```
TraceId → SecureHeaders → CORS → RateLimiter → Logger → Transform → Session → ErrorHandler
```

## 依赖关系

```
@seed/api → @seed/services（registerCoreRoutes 注册共享路由）
         → @seed/kit（auth, hono/middleware, utils）
         → @seed/contracts（schemas, errors, enums）
         → @seed/db（Drizzle schema + provider）
```

## 开发命令

```bash
pnpm nx dev api       # 热重载开发（bun --hot）
pnpm nx build api     # 构建
pnpm nx test api      # 测试
```

## 部署

```bash
# Cloudflare Workers
pnpm nx deploy api    # 通过 wrangler 部署

# 环境：dev / staging / production（见 wrangler.toml）
```

## 与 apps/server 的关系

| 维度   | api（本项目）                 | server            |
| ------ | ----------------------------- | ----------------- |
| 定位   | 80% 无状态请求                | 20% 有状态/长连接 |
| 数据库 | Neon HTTP（无连接池）         | pg TCP（连接池）  |
| 部署   | Cloudflare Workers            | Docker            |
| 入口   | `worker.entry.ts` / `main.ts` | `main.ts`         |
| 共享   | @seed/kit, @seed/db, 中间件   | 同左              |
