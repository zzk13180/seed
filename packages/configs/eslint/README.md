# @seed/eslint-config

> 共享 ESLint 规则模块 + 自定义规则

## AI 参考指引

- **不要直接编辑根目录 `eslint.config.js`** 来添加通用规则，通用规则应放在此包
- 根 `eslint.config.js` 导入此包的规则并按项目类型组合
- 自定义规则在 `custom-rules/` 下实现，遵循 ESLint Rule API

## 导出入口

| 路径 | 内容 |
|------|------|
| `@seed/eslint-config/rules` | `disabledRules` + `regexpRules` + `classMembersOrder` |
| `@seed/eslint-config/custom-rules` | `noIsolatedComments` 自定义规则 |

## 目录结构

```
eslint/
├── rules/
│   ├── index.js                 # barrel export
│   ├── disabled-rules.js        # 全局关闭的规则
│   ├── regexp-rules.js          # 正则相关规则配置
│   └── class-members-order.js   # 类成员排序规则
└── custom-rules/
    ├── index.js                 # barrel export
    └── no-isolated-comments.js  # 禁止类内部孤立注释（配合 sort-class-members 自动修复）
```
