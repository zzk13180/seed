# @seed/ui

> Svelte 5 共享 UI 组件库

## AI 参考指引

- **新建 Svelte 组件时参考此包**：Svelte 5 + SvelteKit 组件开发规范
- 组件在 `src/lib/` 下，通过 Vite library mode 打包输出 ES module
- 目前主要服务于需要 Svelte 组件的项目

## 技术栈

| 维度 | 技术                       |
| ---- | -------------------------- |
| 框架 | Svelte 5 + SvelteKit       |
| 构建 | Vite 7                     |
| 打包 | Vite library mode（`vite.lib.config.js`） |

## 开发命令

```bash
pnpm nx dev ui      # 开发服务器
pnpm nx build ui    # 构建库
```
