# Admin Project (Seed)

## 核心架构设计

通过 Controller + Store 分层设计，让核心业务代码与 UI 框架彻底解耦。

### 核心特点

| 特点         | 说明                                                         |
| ------------ | ------------------------------------------------------------ |
| **框架无关** | 核心逻辑封装在纯 TS Controller 中，迁移框架只需重写 Store 层 |
| **易测试**   | 无需 mock Vue 环境，直接 `new Controller()` 进行单元测试     |
| **强内聚**   | 状态变更逻辑收敛，拒绝逻辑散落在组件各处                     |
| **OOP 友好** | 纯 class 设计可充分利用面向对象特性，提升代码复用性          |

## 架构分层

建议严格遵守单向依赖原则：`View -> Store -> Controller -> Service -> Types`

| 层级           | 文件后缀          | 职责                           | 依赖                       |
| :------------- | :---------------- | :----------------------------- | :------------------------- |
| **View**       | `*.vue`           | UI 渲染与交互事件              | Store                      |
| **Store**      | `*.store.ts`      | 响应式状态托管、HMR、依赖组装  | Controller, Pinia          |
| **Controller** | `*.controller.ts` | **核心业务逻辑**、状态变更     | Types, Service (Interface) |
| **Service**    | `*.service.ts`    | 基础设施实现 (API/Storage/DOM) | Types (Interface)          |
| **Types**      | `*.types.ts`      | 纯类型定义、接口抽象           | 无依赖                     |

- **Controller**: 纯 TypeScript 类，封装核心业务逻辑和数据变迁规则。它不依赖 Vue，只依赖纯粹的数据结构。
- **Store**: **组件级别**的状态容器，负责连接 Vue 视图和 Controller，处理响应式绑定和 HMR（热更新）。每个功能模块拥有独立的 Store。

```
┌─────────────────────────────────────────────────────┐
│  View (*.vue)                                       │
│  ├── 调用 store.controller.method()                 │
│  └── 绑定 store.state.xxx                           │
└────────────────────────┬────────────────────────────┘
                         │ 依赖
┌────────────────────────▼────────────────────────────┐
│  Store (*.store.ts)                                 │
│  ├── reactive(state) ──────────────────┐            │
│  ├── markRaw(controller)               │ 注入       │
│  └── HMR 处理                          │            │
└────────────────────────────────────────┼────────────┘
                                         │
┌────────────────────────────────────────▼────────────┐
│  Controller (*.controller.ts)   【纯 TypeScript】   │
│  ├── 直接修改 state (引用传递，View Auto Update)      │
│  ├── 调用 env.service 执行副作用                    │
│  └── 生命周期: initialize() / dispose()            │
└─────────────────────────────────────────────────────┘
```

## 核心开发指南

### 1. 依赖注入 (DI)

Controller 不应直接 `import` 具体服务实现类，而是通过构造函数注入接口。这不仅解耦了依赖，也让单元测试变得极易编写。

```typescript
// types.ts (定义接口)
export interface UserEnv {
  api: UserApi
  logger: Logger
}

// controller.ts (使用接口)
export class UserController {
  constructor(
    private state: UserState,
    private env: UserEnv,
  ) {}
}

// store.ts (注入实现)
const env = { api: new UserApiImpl(), logger: console }
const controller = new UserController(state, env)
```

### 2. 生命周期管理

对于需要异步初始化（API加载）或资源清理（WebSocket/定时器）的模块，建议继承 `BaseController`。

| 场景 | 推荐基类 | 特性 |
| :-- | :-- | :-- |
| **异步/资源型** | `extends BaseController` | 获得 `initialize()` (竞态保护) 和 `dispose()` 统一管理 |
| **纯逻辑/同步** | 无 (普通类) | 轻量，无额外开销 |

### 3. Service 层封装

所有副作用（HTTP请求、LocalStorage、DOM操作）应封装在 Service 层。

- **API 封装**：避免 Controller 直接调用 `fetch/axios`。
- **UI 库解耦**：Message/Notify 等 UI 反馈也应封装为 Service 接口。

## 标准模块实现参考

以 `views/panel/` 为例，展示一个完整模块的实现结构。

### 1. 接口定义 (`panel.types.ts`)

```typescript
export interface PanelState {
  loading: boolean
  data: any
}
// 环境依赖接口 (抽象)
export interface PanelEnv {
  api: { fetchData(): Promise<any> }
  message: { error(msg: string): void }
}
```

### 2. 业务逻辑 (`panel.controller.ts`)

纯净的业务逻辑，**严禁引入 vue/pinia**。

```typescript
import { BaseController } from '@/core/base.controller'
import type { PanelState, PanelEnv } from './panel.types'

export class PanelController extends BaseController<PanelState, PanelEnv> {
  protected async onInit() {
    this.state.loading = true
    try {
      this.state.data = await this.env.api.fetchData()
    } catch (e) {
      this.env.message.error('加载失败')
    } finally {
      this.state.loading = false
    }
  }

  protected async onDispose() {
    /* 清理逻辑 */
  }
}
```

### 3. 状态装配 (`panel.store.ts`)

Vue/Pinia 胶水层，负责组装所有部件。

**方案 A：markRaw（推荐，简洁优先）**

```typescript
import { defineStore } from 'pinia'
import { reactive, markRaw } from 'vue'
import { PanelController } from './panel.controller'
import { HttpPanelApi, ElementMessage } from './panel.service'

export const usePanelStore = defineStore('panel', () => {
  const env = { api: new HttpPanelApi(), message: new ElementMessage() }
  const state = reactive({ loading: false, data: null })
  const controller = markRaw(new PanelController(state, env))

  return { state, controller }
})
```

**方案 B：shallowRef + HMR（需要 Controller 热更新时）**

当你频繁修改 Controller 逻辑并希望无刷新预览时，可启用此模式：

```typescript
import { reactive, computed, shallowRef } from 'vue'
import { defineStore, acceptHMRUpdate } from 'pinia'
import { PanelController } from './panel.controller'
import type { PanelState, PanelEnv } from './panel.types'

// HMR 替换函数（仅开发环境）
let _hotReplaceController: ((NewClass: typeof PanelController) => void) | null = null

export const usePanelStore = defineStore('panel', () => {
  const env: PanelEnv = {
    /* 依赖注入 */
  }
  const state = reactive<PanelState>({ loading: false, data: null })

  // shallowRef 允许整体替换 controller 引用
  const controllerRef = shallowRef(new PanelController(state, env))

  // 注册 HMR 替换逻辑
  if (import.meta.hot) {
    _hotReplaceController = NewControllerClass => {
      controllerRef.value.dispose() // 清理旧实例
      controllerRef.value = new NewControllerClass(state, env) // 注入相同的 state 和 env
    }
  }

  return { state, controller: controllerRef }
})

// HMR 配置
if (import.meta.hot) {
  import.meta.hot.accept(acceptHMRUpdate(usePanelStore, import.meta.hot))
  import.meta.hot.accept('./panel.controller', newModule => {
    if (newModule?.PanelController && _hotReplaceController) {
      _hotReplaceController(newModule.PanelController)
    }
  })
}
```

**两种方案对比**：

| 特性              | markRaw               | shallowRef + HMR            |
| ----------------- | --------------------- | --------------------------- |
| 复杂度            | 简单                  | 较复杂                      |
| 调用方式          | `controller.method()` | `controller.value.method()` |
| Controller 热更新 | ❌ 需刷新             | ✅ 无刷新                   |
| 适用场景          | 生产环境、稳定模块    | 开发调试阶段                |

> **建议**：大多数情况下使用 `markRaw` 即可。Controller HMR 主要用于开发阶段快速迭代复杂业务逻辑。

### 4. 视图调用 (`PanelView.vue`)

```vue
<script setup lang="ts">
  import { onMounted, onUnmounted } from 'vue'
  import { usePanelStore } from './panel.store'

  const { state, controller } = usePanelStore()

  onMounted(() => controller.initialize())
  onUnmounted(() => controller.dispose())
</script>
```

## 开发约束（LLM enforced）

> 以下规则是**强制性约束**，确保代码的可测试性与解耦。

### 实现顺序

遵循 **“接口先行，实现后置”** 的原则：

```
1. Types      (定义 State 结构、Env 接口、Service 接口)
2. Controller (编写核心业务逻辑，依赖接口不依赖实现)
3. Service    (实现具体的 API 调用、LocalStorage 等基础设施)
4. Store      (组装 State、Controller 和 Service 实现)
5. View       (绑定 State，调用 Controller 方法)
```

### 层级隔离规则

| 文件类型 | ❌ 禁止导入 (依赖反转) | ✅ 允许导入 |
| --- | --- | --- |
| `*.controller.ts` | Vue, Pinia, Router, **具体实现类** | Types, Service 接口, Utils, BaseController |
| `*.types.ts` | 任何运行时代码 (class/func) | 仅类型定义 (interface/type) |
| `*.service.ts` | UI 组件, Controller, Store | Types, 外部库 (axios 等), Utils |
| `*.vue` | Controller (必须通过 Store 访问) | 本模块 Store, UI 组件库 |

### 命名规范

| 类型       | 命名模式             | 示例              |
| ---------- | -------------------- | ----------------- |
| Store      | `use[Module]Store`   | `usePanelStore`   |
| Controller | `[Module]Controller` | `PanelController` |
| Service    | `[Module]Service`    | `PanelService`    |
| State 类型 | `[Module]State`      | `PanelState`      |
| Env 类型   | `[Module]Env`        | `PanelEnv`        |

### 导入规范 (防循环依赖)

- **禁用 Barrel Exports**: 禁止使用 `index.ts` 进行重导出，必须指向具体文件。
- **使用完整路径**:
  - ✅ `import { Logger } from '@/core/logger.service'`
  - ❌ `import { Logger } from '@/core'` (虽然方便，但易导致隐式循环依赖)

### 反模式（禁止事项）

- ❌ **逻辑泄露**：在 `.vue` 或 Pinia `actions` 中编写复杂业务分支（应委托给 Controller）。
- ❌ **直接实例化**：在 Controller 内部使用 `new Service()`（违反依赖注入，导致无法单测）。
- ❌ **越权修改**：在 View 中直接修改 `store.state`（应调用 `controller` 方法表达意图）。
- ❌ **私自通信**：Controller 直接导入其他模块的 Controller（应由 View 层协同或 Service 层通信）。
