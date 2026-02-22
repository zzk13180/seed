# packages/configs/

> 构建工具配置共享包集合 — ESLint / Tailwind / TSConfig

## AI 参考指引

- 这些包**不含运行时代码**，只包含构建/检查工具的配置
- 各 app 和 package 通过 workspace 依赖引用这些配置
- 修改配置时注意影响范围：所有子项目都会受到影响

## 子包

| 包 | Npm 名 | 作用 |
|----|--------|------|
| `eslint/` | `@seed/eslint-config` | ESLint 规则模块（disabled-rules、regexp-rules、class-members-order） + 自定义规则 |
| `tailwind/` | `@seed/tailwind-config` | Tailwind CSS 共享预设（颜色/字体/间距/动画 token） |
| `tsconfig/` | `@seed/tsconfig` | 基础 TypeScript 编译选项（strict 模式、bundler 解析） |
