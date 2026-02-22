# apps/

> 应用层（Layer 3）— 所有可部署/运行的应用集合

## AI 参考指引

- 每个子目录是一个**集合目录**，按平台或运行时分组
- 应用之间**禁止互相依赖**（包括 spa/ 和 native/ 下的子项目）
- 应用只向下依赖 `packages/services`（Layer 2）→ `packages/kit` + `packages/db`（Layer 1）→ `packages/contracts`（Layer 0）
- 新增应用时先通过 `nx generate` 脚手架创建，不要手动复制

## 目录结构

```
apps/
├── api/              # 后端 API 集合（双运行时）
│   ├── edge/         # Serverless API（Hono → Cloudflare Workers, Neon HTTP）
│   └── bun/          # 重型任务（Hono on Bun Docker, Neon TCP）
├── spa/              # Web SPA 集合（不区分桌面/移动端）
│   ├── admin/        # 平台管理后台（Vue + Element Plus + Tailwind）
│   └── console/      # 通用控制台（Vue + Ionic）
├── native/           # Tauri 原生应用集合（桌面 + Android + iOS）
│   └── seed/         # 主应用
├── docs/             # 文档站（Astro SSG）
└── robotics/         # 机器人（ROS1 + Python + C++）
```

## Nx 项目命名

| 目录 | Nx 项目名 |
|------|-----------|
| `api/edge` | `@seed/api-edge` |
| `api/bun` | `@seed/api-bun` |
| `spa/admin` | `@seed/spa-admin` |
| `spa/console` | `@seed/spa-console` |
| `native/seed` | `@seed/native-seed` |
| `docs` | `@seed/docs` |
| `robotics` | `@seed/robotics` |
