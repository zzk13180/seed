# @seed/docs

> Seed 项目文档站，基于 Astro 5 + Starlight。

## AI 参考指引

- **项目文档的 SSOT**：架构设计、编码约定、模板等文档集中在 `src/content/docs/`
- 文档格式为 MDX，支持 Astro 组件嵌入
- 架构文档在 `architecture/` 子目录（backend.mdx, frontend.mdx, deployment.mdx）
- 新增文档在 `src/content/docs/` 下创建 `.mdx` 文件即可自动纳入

## 技术栈

| 维度 | 技术 |
| --- | --- |
| 框架 | Astro 5 (SSG) |
| 主题 | Starlight |
| 内容 | MDX (Markdown + JSX) |

## 目录结构

```
src/content/docs/
├── index.mdx               # 首页
├── getting-started.mdx     # 快速开始
├── conventions.mdx         # 编码约定
├── templates.mdx           # 脚手架模板
└── architecture/
    ├── index.mdx           # 架构全景
    ├── backend.mdx         # 后端架构
    ├── frontend.mdx        # 前端架构
    └── deployment.mdx      # 部署架构
```

## 开发

```bash
pnpm nx dev docs
```

## 构建

```bash
pnpm nx build docs
```
