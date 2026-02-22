# zzk13180_seed_common

> 跨 Tauri 应用共享的 Rust 工具库

## AI 参考指引

- 提供平台检测等通用工具函数
- 被 `apps/native/seed/crates/seed`（Tauri 主 crate）通过 path 依赖引用
- crate 名带 `zzk13180_` 前缀以避免 crates.io 命名冲突

## 功能

| 函数 | 说明 |
|------|------|
| `is_win_11()` | 检测当前系统是否为 Windows 11（通过 build number ≥ 22000） |

## 依赖

| crate | 用途 |
|-------|------|
| `sysinfo` | 系统信息查询 |
