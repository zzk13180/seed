# apps/api — 后端 API 服务集合

> 同一套业务逻辑（@seed/services）的不同部署形态

## AI 参考指引

- **不要在此目录写业务逻辑**：路由处理器在 `packages/services/`，app 层只做组装
- 新增 API 端点先在 `packages/services/` 创建模块，再到 `edge/src/app.ts` 或 `bun/src/app.ts` 注册路由
- edge 和 bun 的区别见下方对比表，根据场景选择部署目标
- 数据库连接方式不同：edge 用 Neon HTTP，bun 用 pg TCP Pool

## 目录结构

```
apps/api/
├── edge/       # Cloudflare Workers (Neon HTTP, 无状态 CRUD)
├── bun/        # Bun Docker (pg TCP, WebSocket/Cron/文件)
└── {new-api}/  # 新增的 API 部署目标
```

## 设计原则

- **业务代码只写一份**：所有模块在 `packages/services/` 定义
- **app 层只做组装**：选择运行时 + 选择 DB 驱动 + 注册路由
- **新增部署目标**：复制 `edge/` 或 `bun/` → 修改 database provider → 注册路由

## edge vs bun

| 维度   | edge                          | bun               |
| ------ | ----------------------------- | ------------------ |
| 定位   | 80% 无状态请求                | 20% 有状态/长连接  |
| 数据库 | Neon HTTP（无连接池）         | pg TCP（连接池）   |
| 部署   | Cloudflare Workers            | Docker             |
| 特有   | wrangler, worker.entry.ts     | WebSocket, Cron    |
| 认证   | Bearer Token                  | Bearer Token       |
