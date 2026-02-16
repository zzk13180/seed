# CONVENTIONS.md — 代码约定

> 本文档定义种子项目的代码约定。AI Agent 生成新代码时，必须遵守以下规则。

---

## 1. 命名约定

### 文件命名

| 类型         | 格式                   | 示例                                    |
| ------------ | ---------------------- | --------------------------------------- |
| 模块文件     | `kebab-case.{role}.ts` | `user.controller.ts`, `user.service.ts` |
| Zod Schema   | `{name}.schema.ts`     | `user.schema.ts`, `common.schema.ts`    |
| 视图对象     | `{name}.vo.ts`         | `user.vo.ts`                            |
| 中间件       | `{name}.middleware.ts` | `auth-guard.middleware.ts`              |
| 工具函数     | `{name}.util.ts`       | `sql.util.ts`, `date.util.ts`           |
| 枚举         | `{name}.enum.ts`       | `user.enum.ts`, `response-code.enum.ts` |
| Vue 组件     | `PascalCase.vue`       | `UserManagement.vue`                    |
| Vue 模块文件 | `kebab-case.{role}.ts` | `user-management.controller.ts`         |
| 配置文件     | `{name}.config.ts`     | `env.config.ts`, `auth.config.ts`       |

### 变量/类/函数命名

| 类型       | 格式                                    | 示例                                    |
| ---------- | --------------------------------------- | --------------------------------------- |
| 类         | PascalCase                              | `UserService`, `UserRepository`         |
| 函数/方法  | camelCase                               | `findPage()`, `createUser()`            |
| 常量       | UPPER_SNAKE_CASE                        | `RESPONSE_CODE`, `HTTP_STATUS`          |
| 枚举       | PascalCase (名) + UPPER_SNAKE_CASE (值) | `UserStatus.ENABLED`                    |
| 接口       | PascalCase, 无 I 前缀                   | `UserVO`, `PageResult`                  |
| 类型别名   | PascalCase                              | `UserCreateDto`, `LoginDto`             |
| Zod Schema | camelCase + Schema 后缀                 | `userCreateSchema`, `pageRequestSchema` |

---

## 2. 后端模块约定

### 新增模块清单

创建 `packages/services/src/{name}/` 时，至少包含以下文件：

```
packages/services/src/{name}/
├── {name}.controller.ts    # 必须
├── {name}.service.ts       # 必须
├── {name}.repository.ts    # 有数据库操作时必须
├── {name}.schema.ts        # 有 Zod 验证时必须
└── {name}.vo.ts            # 有 Entity→VO 转换时必须
```

### Controller 规范

```typescript
// ✅ 正确：Controller 只编排，不包含业务逻辑
.get('/', zValidator('query', userQuerySchema), async (c) => {
  const query = c.req.valid('query')
  const result = await userService.findPage(query)
  return c.json(result)
})

// ❌ 错误：Controller 包含业务逻辑
.get('/', async (c) => {
  const users = await db().select().from(user).where(...)  // 直接操作 ORM
  const filtered = users.filter(u => u.status === 1)       // 业务逻辑
  return c.json(filtered)
})
```

### Service 规范

- 抛出 `BusinessError` 子类（`NotFoundError`, `ConflictError` 等），不手写 HTTP 状态码
- 返回 VO 类型（经 `toXxxVO()` 转换），不直接返回 Entity
- 不引用 Hono Context，不操作 HTTP 请求/响应

### Repository 规范

- 方法命名：`findById`, `findPage`, `findAll`, `update`, `softDelete`, `batchSoftDelete`
- 排序字段使用 **白名单**（`SORTABLE_FIELDS`），防止 ORDER BY 注入
- LIKE 查询使用 `escapeLikeString()` 转义特殊字符

---

## 3. Zod Schema 约定

### 位置规则

| Schema 类型 | 存放位置                                        | 说明                           |
| ----------- | ----------------------------------------------- | ------------------------------ |
| 跨模块共享  | `@seed/contracts/schemas/`                      | 通用分页、响应结构、用户、认证 |
| 模块专用    | `packages/services/src/{name}/{name}.schema.ts` | 路径参数、模块内部验证         |
| 前端表单    | `@seed/contracts/schemas/` → 前端 re-export     | 前后端共用同一份验证           |

### 编写规范

```typescript
// ✅ Zod schema 作为 SSOT，类型从 schema 推导
export const userCreateSchema = z.object({ ... })
export type UserCreateDto = z.infer<typeof userCreateSchema>

// ❌ 手写 interface 再手写 schema（重复维护）
export interface UserCreateDto { ... }
export const userCreateSchema = z.object({ ... }) // 二次声明
```

### 分页查询扩展

```typescript
// 使用 pageRequestSchema.extend() 添加业务搜索字段
import { pageRequestSchema } from '@seed/contracts/schemas/common'

export const userQuerySchema = pageRequestSchema.extend({
  username: z.string().optional(),
  status: z.coerce.number().int().min(0).max(1).optional(),
})
```

---

## 4. 错误处理约定

### 错误类继承体系

```
BusinessError (base)
├── ValidationError     (422)  — 参数校验失败
├── UnauthorizedError   (401)  — 未认证
├── ForbiddenError      (403)  — 无权限
├── NotFoundError       (404)  — 资源不存在
└── ConflictError       (409)  — 资源冲突
```

### 使用规范

```typescript
// ✅ 使用语义化工厂方法
throw new NotFoundError('User', id)
throw ConflictError.emailExists(email)
throw UnauthorizedError.accountDisabled()

// ❌ 手写 HTTP 状态码
throw new Error('Not Found') // 无状态码
c.json({ error: 'Not Found' }, 404) // 在 Service 层操作 HTTP
```

### 所有错误由 `errorHandlerMiddleware` 统一处理

Service/Repository 只管抛异常，不负责构建错误响应。

---

## 5. 导入约定

### 优先级

```typescript
// 1. Node.js 内置模块
import * as path from 'node:path'

// 2. 第三方依赖
import { Hono } from 'hono'
import { z } from 'zod'

// 3. Workspace 共享包
import { userCreateSchema } from '@seed/contracts/schemas/user'
import { BusinessError } from '@seed/contracts/errors'
import { user } from '@seed/db/schema'

// 4. 项目内部模块
import { UserService } from './user.service'
import { db } from '../../database/database.provider'
```

### 子路径导入

优先使用子路径导入以获得更好的 tree-shaking：

```typescript
// ✅ 精确子路径
import { userCreateSchema } from '@seed/contracts/schemas/user'
import { ResponseCode } from '@seed/contracts/errors/response-code'

// ⚠️ 可用但不推荐（barrel export，可能打包更多代码）
import { userCreateSchema, ResponseCode } from '@seed/contracts'
```

---

## 6. 前端模块约定

### Controller 层

- **纯 TS 类**，严禁导入 Vue / Pinia / Hono
- 通过构造函数接收 `state` (reactive) 和 `deps` (DI)
- 继承 `BaseController`，使用 `onInit()` / `onDispose()` 生命周期
- 所有数据变更通过修改 `this.state` 实现

### Store 层

- 使用 `defineStore` 的 setup 语法
- 负责 DI 组装：实例化 Service → 构造 deps → 创建 Controller
- Controller 必须用 `markRaw()` 包裹（避免深响应式）
- 暴露 `{ state, controller }` — view 层只用这两个

### Service 层

- 实现 types.ts 中定义的接口（如 `UsersApi`）
- 内部可使用 Hono RPC / fetch / 任何 HTTP client
- 通过 DI 注入到 Controller，Controller 不关心实现细节

---

## 7. 测试约定

| 层级 | 测试方式 | 位置 |
| --- | --- | --- |
| Controller (后端) | 集成测试 (Hono `app.request`) | `packages/services/src/{name}/__tests__/{name}.controller.test.ts` |
| Service | 单元测试 (mock Repository) | `packages/services/src/{name}/__tests__/{name}.service.test.ts` |
| Repository | 集成测试 (测试数据库) | `packages/services/src/{name}/__tests__/{name}.repository.test.ts` |
| Controller (前端) | 单元测试 (mock deps) | `views/{module}/__tests__/` |
| Shared packages | 单元测试 | `packages/{name}/src/**/*.test.ts` |

---

## 8. Git 提交约定

```
<type>(<scope>): <subject>

feat(api):      新增用户批量删除接口
fix(admin):     修复表格分页跳转问题
refactor(kit): 迁移 Zod schemas 到 @seed/contracts
chore(nx):      更新 namedInputs 缓存配置
docs:           更新 ARCHITECTURE.md
```

type: `feat` | `fix` | `refactor` | `chore` | `docs` | `test` | `style` | `perf` | `ci`
