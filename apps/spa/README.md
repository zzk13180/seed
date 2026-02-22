# apps/spa — Web SPA 应用集合

> 包含所有 Web 单页应用，不区分桌面端和移动端

## AI 参考指引

- 每个 SPA 都遵循 **Controller + Store** 分层架构，详见 `admin/README.md`
- admin 用 Element Plus，console 用 Ionic 8，但共享同一套架构模式
- 新建 SPA 复制现有模板，详见下方“新建 Web SPA”步骤
- API 调用统一通过 Hono RPC (`hono/client`)，不使用 axios/fetch

## 目录结构

```
apps/spa/
├── admin/      # 平台管理后台（Vue + Element Plus + TailwindCSS）
├── console/    # 通用控制台（Vue + Ionic）
└── {new-app}/  # 新增的 Web SPA 按此模式创建
```

## 新建 Web SPA

1. 复制 `admin/` 或 `console/` 为模板
2. 修改 `package.json` 的 `name`（格式：`@seed/spa-{name}`）
3. 修改 `project.json` 的 `name` 和 `cwd` 路径
4. 选择 UI 框架：Element Plus（admin 类）或 Ionic（通用类）

## 共享基础

所有 SPA 共享：

- `@seed/kit/frontend`：BaseController + Logger
- `@seed/contracts`：Zod schemas + 类型
- **Controller + Store** 分层架构模式
- **Bearer Token** 统一认证模式
