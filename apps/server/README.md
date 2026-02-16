# @seed/server

> Hono + Bun 重型任务 API 服务（Docker 部署，处理 20% 有状态/长连接请求）

## AI 参考指引

- **新建 Hono API 项目时参考此项目**：模块结构（Controller → Service → Repository）、中间件链、Better Auth 集成
- 与 `apps/api/` 共享完全相同的架构模式，区别仅在数据库连接方式（TCP 长连接 vs HTTP）
- 中间件通过 `@seed/kit/hono/middleware` 共享，**不要**在 app 内新建中间件

## 技术栈

| 维度   | 技术                                  |
| ------ | ------------------------------------- |
| 运行时 | Bun                                   |
| 框架   | Hono                                  |
| 认证   | Better Auth（共享 `@seed/kit/auth`）  |
| ORM    | Drizzle ORM（共享 `@seed/db`）        |
| 数据库 | PostgreSQL TCP（`pg` 驱动，长连接池） |
| 验证   | Zod（共享 `@seed/contracts/schemas`） |
| 部署   | Docker（docker-compose）              |

## 目录结构

```
src/
├── app.ts                  # Hono 应用工厂（registerCoreRoutes + server-only 路由）
├── main.ts                 # Bun 入口（优雅关机）
├── config/                 # 环境变量（Zod 验证）+ Better Auth 实例
└── database/               # pg Pool TCP 连接 + Drizzle 实例
```

## 依赖关系

```
@seed/server → @seed/services（registerCoreRoutes + createTauriUpdaterRoutes）
             → @seed/kit（auth, hono/middleware, utils）
             → @seed/contracts（schemas, errors, enums）
             → @seed/db（Drizzle schema）
```

## 开发命令

```bash
pnpm nx dev server     # 热重载开发（bun --hot）
pnpm nx build server   # 构建
pnpm nx test server    # 测试
```

## Docker 部署

```bash
cd apps/server/docker
docker compose up -d          # 启动（server + postgres + redis）
docker compose down           # 停止
docker compose -f docker-compose.prod.yml up -d  # 生产环境
```
