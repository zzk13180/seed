# apps/native — Tauri 原生应用集合

> 包含所有基于 Tauri 2 构建的原生应用（桌面端 + Android + iOS）

## AI 参考指引

- Tauri 前端代码在 `seed/src/`（Vue 3），Rust 代码在 `seed/crates/`
- Rust crate 名称必须在根 `Cargo.toml` 的 `workspace.members` 中注册
- 平台权限/原生 API 在 Rust 侧，通过 Tauri Commands 暴露给前端
- 认证统一使用 Bearer Token，Token 存储在系统安全存储中

## 目录结构

```
apps/native/
├── seed/       # 主应用 — Tauri 2 (桌面 + Android + iOS)
└── {new-app}/  # 新增的 Tauri 应用按此模式创建
```

## 新建原生应用

1. 复制 `seed/` 为模板
2. 修改 `package.json` 的 `name`（格式：`@seed/native-{name}`）
3. 修改 `project.json` 的 `name` 和 `cwd` 路径
4. 修改 `crates/{name}/Cargo.toml` 的 crate 名称
5. 在根 `Cargo.toml` 的 `workspace.members` 添加新 crate 路径

## 认证

所有原生应用统一使用 **Bearer Token** 模式，Token 存储在系统安全存储中。
