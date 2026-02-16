# @seed/desktop

> Tauri 2 + Rust 桌面客户端应用（多平台构建）

## AI 参考指引

- **新建 Tauri 桌面应用时参考此项目**：Tauri 2 插件配置、多平台构建脚本、Cargo workspace 组织
- Rust 应用代码在 `crates/seed/`（Tauri 主 crate），共享 Rust 代码在 workspace 根 `crates/seed-common/`
- 前端打包产物由 `apps/admin/` 或其他 web app 提供（Tauri 嵌入 WebView）
- 应用自动更新通过 `tauri-plugin-updater` + `apps/api/ tauri-updater` 模块实现

## 技术栈

| 维度     | 技术                                                             |
| -------- | ---------------------------------------------------------------- |
| 桌面框架 | Tauri 2                                                          |
| 系统语言 | Rust (edition 2021, MSRV 1.77.2)                                 |
| 构建     | Cargo workspace                                                  |
| 插件     | opener, log, process, updater, single-instance, window-state, os |
| 发布优化 | LTO (thin), strip symbols, panic=abort                           |

## 目录结构

```
├── crates/
│   └── seed/           # 主应用 crate（Tauri commands + 插件注册）
├── src/                # 前端资源（JS/TS 入口）
├── package.json        # Nx 项目配置
└── project.json        # Nx targets（dev, build）
```

> **注意**: Cargo workspace 根在 monorepo 顶层 `Cargo.toml`，共享 Rust crate 在 `crates/seed-common/`

## 多平台构建

```bash
# macOS
pnpm nx build desktop   # 默认当前平台
# 脚本支持：universal, aarch64, x64

# Windows: x64, arm64
# Linux: x64, arm64
```

## VS Code 配置

```json
// .vscode/settings.json
{
  "rust-analyzer.linkedProjects": ["./Cargo.toml"]
}
```

## 开发命令

```bash
pnpm nx dev desktop     # Tauri 开发模式（前端热重载 + Rust 编译）
pnpm nx build desktop   # 构建可分发安装包
```
