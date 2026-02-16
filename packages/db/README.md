# @seed/db

> Drizzle ORM 数据库层 — Schema 定义 + 连接 Provider

## AI 参考指引

- **新增数据库表时参考此包**：所有 Drizzle schema 集中在 `src/schema/`
- Better Auth 的内置表（user, session, account, verification）已在 `auth.schema.ts` 定义，不要重复创建
- `provider.ts` 提供 Neon HTTP 连接工厂，`apps/server/` 因需要 TCP 长连接自行建立 `pg Pool`
- 迁移文件由 `drizzle-kit` 自动生成到 `drizzle/` 目录

## 导出入口

| 路径                | 内容                              | 使用方      |
| ------------------- | --------------------------------- | ----------- |
| `@seed/db`          | 主入口                            | api, server |
| `@seed/db/schema`   | 全部 Drizzle 表定义               | api, server |
| `@seed/db/provider` | Neon HTTP 连接 + Drizzle 实例工厂 | api         |

## 目录结构

```
src/
├── index.ts                # 主入口
├── provider.ts             # Neon HTTP drizzle 连接工厂
└── schema/
    ├── index.ts            # barrel export
    └── auth.schema.ts      # Better Auth 内置表（user, session, account, verification）
```

## 技术栈

| 维度   | 技术                                          |
| ------ | --------------------------------------------- |
| ORM    | Drizzle ORM 0.43                              |
| 数据库 | Neon PostgreSQL（`@neondatabase/serverless`） |
| 迁移   | drizzle-kit                                   |

## 依赖关系

```
@seed/db ← @seed/api（schema + provider）
         ← @seed/server（仅 schema，自建 pg Pool 连接）
```

## 数据库命令

```bash
pnpm nx db:generate api   # 生成迁移文件
pnpm nx db:migrate api    # 执行迁移
pnpm nx db:push api       # 推送 schema（开发用）
pnpm nx db:studio api     # Drizzle Studio GUI
```

## 添加新表的规范

1. 在 `src/schema/` 下创建 `xxx.schema.ts`
2. 使用 Drizzle 的 `pgTable()` 定义表
3. 在 `src/schema/index.ts` re-export
4. 运行 `pnpm nx db:generate api` 生成迁移
5. 运行 `pnpm nx db:migrate api` 执行迁移
