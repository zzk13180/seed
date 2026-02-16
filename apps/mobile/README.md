# @seed/mobile

> Ionic + Vue 移动端 Web 应用

## AI 参考指引

- **新建移动端项目时参考此项目**：Ionic 8 + Vue 3 + Pinia 的标准 SPA 架构
- 路由配置在 `router.ts`，使用 Ionic 的 `IonRouterOutlet`
- 状态管理用 Pinia store（`stores/` 目录）
- 与 `apps/admin/` 共享 `@seed/kit`（schemas, enums），但 UI 框架不同（Ionic vs Element Plus）

## 架构模式

遵循 monorepo 统一的 **Controller + Store** 分层架构：

| 层级 | 位置 | 说明 |
| --- | --- | --- |
| **App Controller** | `stores/app.controller.ts` | extends `BaseController<State, AppDeps>`，管理 RxJS 订阅和生命周期 |
| **App Store** | `stores/app.store.ts` | Pinia `defineStore`，桥接响应式 state 与 Controller |
| **Views** | `views/*.vue` | 消费 store 中的响应式 state |

**复杂度分级策略**：

- 当前视图为早期阶段，使用 **单文件 .vue**（无需 per-module 五件套）
- 当视图业务逻辑增长后，按 admin 的五件套模式拆分（types + controller + service + store + vue）
- `BaseController` 已就绪于 `@seed/kit/frontend`，新增模块可直接继承

## 技术栈

| 维度      | 技术              |
| --------- | ----------------- |
| 框架      | Vue 3.5 + Ionic 8 |
| 构建      | Vite 7            |
| 状态管理  | Pinia 3           |
| 手势/动画 | Swiper            |
| 部署      | Docker + PM2      |

## 目录结构

```
src/
├── App.vue             # 根组件
├── main.ts             # 入口（Ionic + Pinia 初始化）
├── router.ts           # 路由配置
├── assets/             # 静态资源
├── components/         # 通用组件
├── layout/             # 布局组件
├── stores/             # Pinia stores
├── styles/             # 全局样式
└── views/              # 页面视图
```

## 依赖关系

```
@seed/mobile → @seed/kit（schemas, enums）
             → @seed/types（全局类型声明）
```

## 开发命令

```bash
pnpm nx dev mobile      # 开发服务器
pnpm nx build mobile    # 构建（production）
pnpm nx preview mobile  # 预览构建产物
```
