# 🚀 Seed Server API (NestJS)

基于 NestJS 11 构建的企业级后端 API 服务。

## 技术栈

- **框架**: NestJS 11
- **语言**: TypeScript 5
- **认证**: JWT + Passport
- **ORM**: TypeORM
- **缓存**: Redis + Cache Manager
- **数据库**: MySQL 8
- **API 文档**: Swagger (OpenAPI 3)
- **验证**: class-validator + class-transformer

## 目录结构

```
src/
├── common/                    # 公共模块
│   ├── database/              # 数据库配置
│   ├── decorators/            # 自定义装饰器
│   │   ├── get-user.decorator.ts
│   │   ├── permissions.decorator.ts
│   │   ├── public.decorator.ts
│   │   └── roles.decorator.ts
│   ├── dto/                   # 通用 DTO
│   │   ├── page-request.dto.ts
│   │   ├── page-result.dto.ts
│   │   └── response.dto.ts
│   ├── entities/              # 基础实体
│   │   └── base.entity.ts
│   ├── enums/                 # 枚举定义
│   │   └── user.enum.ts
│   ├── filters/               # 异常过滤器
│   ├── guards/                # 守卫
│   │   ├── jwt-auth.guard.ts
│   │   ├── permissions.guard.ts
│   │   └── roles.guard.ts
│   ├── interceptors/          # 拦截器
│   ├── interfaces/            # 接口定义
│   │   └── auth.interface.ts
│   ├── pipes/                 # 管道
│   ├── redis/                 # Redis 模块
│   └── strategies/            # Passport 策略
│       └── jwt.strategy.ts
│
├── config/                    # 配置
│   ├── configuration.ts       # 配置加载
│   └── validation.schema.ts   # 环境变量验证
│
├── modules/                   # 业务模块
│   ├── auth/                  # 认证模块
│   │   ├── auth.controller.ts
│   │   ├── auth.module.ts
│   │   ├── auth.service.ts
│   │   ├── dto/
│   │   └── vo/
│   ├── health/                # 健康检查
│   └── user/                  # 用户模块
│       ├── user.controller.ts
│       ├── user.module.ts
│       ├── user.service.ts
│       ├── dto/
│       ├── entities/
│       └── vo/
│
├── app.module.ts              # 应用根模块
└── main.ts                    # 入口文件
```

## 环境变量

| 变量名 | 描述 | 默认值 |
| --- | --- | --- |
| `NODE_ENV` | 运行环境 | `development` |
| `PORT` | 服务端口 | `3003` |
| `API_PREFIX` | API 前缀 | `api` |
| `JWT_SECRET` | JWT 密钥 | - |
| `JWT_ACCESS_TOKEN_EXPIRY` | 访问令牌过期时间 | `1h` |
| `JWT_REFRESH_TOKEN_EXPIRY` | 刷新令牌过期时间 | `7d` |
| `DB_HOST` | 数据库主机 | `localhost` |
| `DB_PORT` | 数据库端口 | `3306` |
| `DB_USERNAME` | 数据库用户名 | `root` |
| `DB_PASSWORD` | 数据库密码 | - |
| `DB_DATABASE` | 数据库名称 | `seed` |
| `DB_SYNC` | 自动同步表结构 | `false` |
| `REDIS_HOST` | Redis 主机 | `localhost` |
| `REDIS_PORT` | Redis 端口 | `6379` |
| `REDIS_PASSWORD` | Redis 密码 | - |

## 开发命令

```bash
# 安装依赖
pnpm install

# 开发模式
pnpm dev

# 构建
pnpm build

# 生产模式运行
pnpm start:prod
```

## API 文档

启动服务后访问 Swagger UI：

```
http://localhost:3003/api/docs
```

## 核心特性

### 1. 统一响应格式

所有 API 返回统一的响应格式：

```json
{
  "code": 200,
  "message": "Success",
  "data": { ... },
  "timestamp": 1640000000000
}
```

### 2. JWT 认证

- 使用 `@Public()` 装饰器标记公开接口
- 使用 `@GetUser()` 装饰器获取当前用户
- 使用 `@Roles()` 装饰器进行角色控制
- 使用 `@Permissions()` 装饰器进行权限控制

### 3. 分页查询

```typescript
// 继承 PageRequestDto 实现分页查询
export class UserQueryDto extends PageRequestDto {
  username?: string
}

// 返回分页结果
PageResultDto.create(list, total, page, pageSize)
```

### 4. 软删除

所有实体继承 `BaseEntity`，自动包含软删除功能。

## 测试

项目包含完善的单元测试和集成测试，使用 Jest 作为测试框架。

### 运行测试命令

```bash
# 运行所有单元测试
pnpm test

# 监听模式运行测试
pnpm test:watch

# 运行测试并生成覆盖率报告
pnpm test:cov

# 调试模式运行测试
pnpm test:debug

# 运行端到端测试
pnpm test:e2e
```

### 测试覆盖范围

测试用例覆盖以下模块：

| 模块 | 测试文件 | 测试内容 |
| --- | --- | --- |
| **UserService** | `user.service.spec.ts` | 用户增删改查、分页、密码验证 |
| **UserController** | `user.controller.spec.ts` | 用户相关 API 接口 |
| **AuthService** | `auth.service.spec.ts` | 登录、登出、令牌刷新、登录次数限制 |
| **AuthController** | `auth.controller.spec.ts` | 认证相关 API 接口 |
| **HealthController** | `health.controller.spec.ts` | 健康检查接口 |
| **HealthService** | `health.service.spec.ts` | 健康检查服务 |
| **ResponseDto** | `response.dto.spec.ts` | 统一响应格式 |
| **PageResultDto** | `page-result.dto.spec.ts` | 分页结果 |
| **PageRequestDto** | `page-request.dto.spec.ts` | 分页请求 |
| **RolesGuard** | `roles.guard.spec.ts` | 角色守卫 |
| **PermissionsGuard** | `permissions.guard.spec.ts` | 权限守卫 |

### 测试统计

- **测试套件**: 11 个
- **测试用例**: 82 个
- **代码覆盖率**: 
  - 核心服务 > 90%
  - 控制器 100%
  - DTO 100%
  - 实体 100%
