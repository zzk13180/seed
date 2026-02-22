# @seed/contracts

> Layer 0 契约层 — Zod schemas / 枚举 / 错误类（纯类型 + 纯数据，零运行时依赖）

## AI 参考指引

- **所有跨模块共享的类型定义都应放在这里**，不要在 services 或 app 中重复定义
- 唯一依赖是 `zod`，任何包都可安全引用
- Schema 是 SSOT（Single Source of Truth）：类型从 schema 推导 `type XxxDto = z.infer<typeof xxxSchema>`
- 模块专用 schema 放在 `packages/services/src/{name}/{name}.schema.ts`，不要放这里
- 错误类使用工厂方法：`throw new NotFoundError('User', id)`

## 导出入口

| 路径 | 内容 |
|------|------|
| `@seed/contracts` | 主入口（全部 re-export） |
| `@seed/contracts/schemas` | 全部 Zod schemas |
| `@seed/contracts/schemas/user` | 用户相关 schema |
| `@seed/contracts/schemas/auth` | 认证相关 schema |
| `@seed/contracts/schemas/common` | 通用 schema（分页、排序等） |
| `@seed/contracts/enums` | 全部枚举（UserStatus 等） |
| `@seed/contracts/errors` | BusinessError 及其子类 |
| `@seed/contracts/errors/response-code` | ResponseCode 枚举 + HTTP 状态码映射 |

## 目录结构

```
src/
├── index.ts            # barrel export
├── schemas/
│   ├── index.ts        # schemas barrel
│   ├── user.schema.ts  # 用户 CRUD schemas
│   ├── auth.schema.ts  # 认证 schemas
│   └── common.schema.ts # 分页、排序等通用 schemas
├── enums/
│   ├── index.ts
│   └── user.enum.ts    # UserStatus 等枚举
└── errors/
    ├── business.error.ts      # BusinessError + 子类（NotFoundError, ConflictError 等）
    └── response-code.enum.ts  # ResponseCode 枚举 + mapCodeToHttpStatus()
```

## 错误类层级

```
BusinessError (base)
├── ValidationError     (422)
├── UnauthorizedError   (401)
├── ForbiddenError      (403)
├── NotFoundError       (404)
└── ConflictError       (409)
```
