# @seed/types

> 全局 TypeScript 类型声明（`.d.ts`）

## AI 参考指引

- **放置全局类型增强和第三方库类型补丁的地方**
- 仅包含 `.d.ts` 文件，无运行时代码
- 大部分 TypeScript 项目通过 `tsconfig.json` 引用本包
- 如需添加新的全局类型（如 `ImportMeta` 扩展），在此包中创建

## 文件说明

| 文件                    | 内容                                    |
| ----------------------- | --------------------------------------- |
| `index.d.ts`            | 统一引用入口（triple-slash references） |
| `vite-env.d.ts`         | Vite 环境变量类型（`ImportMeta`）       |
| `vue-global.d.ts`       | Vue 全局组件类型增强                    |
| `diff-match-patch.d.ts` | diff-match-patch 库类型补丁             |
| `tailwind-config.d.ts`  | Tailwind 配置类型                       |

## 导出入口

| 路径                     | 内容               |
| ------------------------ | ------------------ |
| `@seed/types`            | 主入口（所有声明） |
| `@seed/types/vite-env`   | 仅 Vite 环境类型   |
| `@seed/types/vue-global` | 仅 Vue 全局类型    |
