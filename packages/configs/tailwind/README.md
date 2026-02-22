# @seed/tailwind-config

> 共享 Tailwind CSS 预设 — 统一的设计 token（颜色 / 字体 / 间距 / 动画）

## AI 参考指引

- 使用 Tailwind CSS v4+，配置以 JS preset 形式共享
- 颜色使用 CSS 自定义属性（`var(--seed-color-*)`），支持运行时主题切换
- 当前仅 `apps/spa/admin` 使用 Tailwind

## 使用方式

```js
// tailwind.config.ts
import seedPreset from '@seed/tailwind-config/preset'

export default {
  presets: [seedPreset],
  content: ['./src/**/*.{vue,ts}'],
}
```

## 导出入口

| 路径                           | 内容                                           |
| ------------------------------ | ---------------------------------------------- |
| `@seed/tailwind-config`        | `withSeedPreset()` 工具函数 + `seedPreset`     |
| `@seed/tailwind-config/preset` | Tailwind 预设对象（颜色 / 字体 / 间距 / 动画） |

## 文件

```
tailwind/
├── index.js    # withSeedPreset() 工具函数 + re-export
└── preset.js   # Tailwind 预设（设计 token 定义）
```
