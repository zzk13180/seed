# @seed/tsconfig

> 共享 TypeScript 编译配置 — 所有 TS 项目的基础 tsconfig

## AI 参考指引

- **所有 TS 项目的 `tsconfig.json` 都应 extends 此配置**
- 关键设置：`strict: true`、`moduleResolution: "bundler"`、`noEmit: true`（打包交给 Vite/esbuild）
- 全局类型声明通过 `types: ["@seed/types"]` 注入
- 项目可按需覆盖 `compilerOptions`，但不应降低严格性

## 使用方式

```jsonc
// 各项目的 tsconfig.json
{
  "extends": "@seed/tsconfig/tsconfig.json",
  "compilerOptions": {
    "paths": { "@/*": ["./src/*"] }
  },
  "include": ["src"]
}
```

## 核心配置说明

| 选项 | 值 | 说明 |
|------|-----|------|
| `target` | `esnext` | 最新 ES 特性 |
| `module` | `esnext` | ES 模块，支持 tree-shaking |
| `moduleResolution` | `bundler` | 适配 Vite/esbuild |
| `strict` | `true` | 全部严格检查 |
| `noUncheckedIndexedAccess` | `true` | 索引访问自动加 `undefined` |
| `noEmit` | `true` | 仅类型检查，不输出文件 |
