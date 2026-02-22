# crates/

> Rust 共享 crate 集合 — Cargo workspace 成员

## AI 参考指引

- 根 `Cargo.toml` 是 workspace 入口，所有 Rust crate（包括 `apps/native/seed/crates/seed`）都注册在其 `members` 中
- 此目录下的 crate 被多个 Rust 项目共享
- 版本号和公共依赖在 workspace `Cargo.toml` 中统一管理

## 目录结构

```
crates/
└── seed-common/    # 跨 Tauri 应用共享的 Rust 工具库
```
