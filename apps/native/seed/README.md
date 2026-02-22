# @seed/native-seed

> Tauri 2 原生应用 — 桌面端 (macOS/Windows/Linux) + 移动端 (Android/iOS)

## AI 参考指引

- **新建 Tauri 原生应用时参考此项目**：Tauri 2 多平台构建、插件配置、Cargo workspace 组织
- Rust 应用代码在 `crates/seed/`（Tauri 主 crate），共享 Rust 代码在 workspace 根 `crates/seed-common/`
- 前端打包产物由 `apps/spa/admin/` 或其他 Web SPA 提供（Tauri 嵌入 WebView）
- 应用自动更新通过 `tauri-plugin-updater` + `apps/api/ tauri-updater` 模块实现
- 认证统一使用 Bearer Token 模式（全端一致），Token 存储在系统安全存储

## 技术栈

| 维度     | 技术                                                             |
| -------- | ---------------------------------------------------------------- |
| 原生框架 | Tauri 2（桌面 + Android + iOS）                                  |
| 系统语言 | Rust (edition 2021, MSRV 1.77.2)                                 |
| 构建     | Cargo workspace                                                  |
| 插件     | opener, log, process, updater, single-instance, window-state, os |
| 发布优化 | LTO (full), strip symbols, panic=abort                           |

## 目录结构

```
apps/native/seed/
├── crates/
│   └── seed/           # 主应用 crate（Tauri commands + 插件注册）
│       └── src/
│           ├── main.rs     # 桌面端入口
│           ├── lib.rs      # AppBuilder + Tauri 配置
│           └── mobile.rs   # 移动端入口
├── src/                # 前端资源（JS/TS 入口）
├── package.json        # Nx 项目配置
└── project.json        # Nx targets（dev, build, android:*, ios:*）
```

> **注意**: Cargo workspace 根在 monorepo 顶层 `Cargo.toml`，共享 Rust crate 在 `crates/seed-common/`

## Nx 任务

```bash
# 桌面端
pnpm nx run @seed/native-seed:dev         # 开发模式（前端热重载 + Rust 编译）
pnpm nx run @seed/native-seed:build       # 构建可分发安装包

# Android
pnpm nx run @seed/native-seed:android:dev
pnpm nx run @seed/native-seed:android:build

# iOS
pnpm nx run @seed/native-seed:ios:dev
pnpm nx run @seed/native-seed:ios:build
```

## 集合目录说明

`apps/native/` 是 Tauri 原生应用的集合目录。新增原生应用时在此目录创建新子项目：

```
apps/native/
├── seed/       # 主应用（当前）
└── {new-app}/  # 新增的 Tauri 应用
```

## VS Code 配置

```json
// .vscode/settings.json
{
  "rust-analyzer.linkedProjects": ["./Cargo.toml"]
}
```
