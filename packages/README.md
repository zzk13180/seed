# packages/

> 共享包集合 — 按架构分层组织，所有应用共用的代码都在这里

## AI 参考指引

- 依赖方向严格单向：`apps/ → services → kit + db → contracts`
- 新增共享代码前先确认放在哪一层，不要破坏分层规则
- `configs/` 是构建工具配置共享包，不含运行时代码
- `libs/` 是纯类型声明包

## 分层架构

```
Layer 2  services/       业务模块（user / auth / health / tauri-updater）
  ↓
Layer 1  kit/ + db/      基础设施（中间件 / ORM / BaseController / Auth 工厂）
  ↓
Layer 0  contracts/      契约层（Zod schemas / 枚举 / 错误类，仅依赖 zod）
```

## 目录结构

```
packages/
├── contracts/        # @seed/contracts — Layer 0 纯类型 + Zod schemas
├── db/               # @seed/db — Layer 1 Drizzle ORM schema + 连接工厂
├── kit/              # @seed/kit — Layer 1 Hono 中间件 + 前端 BaseController + 工具
├── services/         # @seed/services — Layer 2 业务模块（三层架构）
├── ui/               # @seed/ui — Svelte 5 Web Components（独立，无层级）
├── configs/          # 构建工具配置集合（ESLint / Tailwind / TSConfig）
└── libs/             # 纯类型声明集合
    └── types/        # @seed/types — 全局 .d.ts 声明
```
