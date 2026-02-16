export const BASE = 'seed'

export const SITE = {
  title: 'Seed Docs',
  description: '企业级全栈多平台开发模板文档',
  defaultLanguage: 'zh-cn',
  author: 'Seed Team',
  repo: 'https://github.com/your-org/seed',
}

// 支持的语言配置
export const LANGUAGES = {
  'zh-cn': {
    label: '中文',
    lang: 'zh-CN',
  },
  en: {
    label: 'English',
    lang: 'en-US',
  },
} as const

export type LanguageCode = keyof typeof LANGUAGES
export const LANGUAGE_CODES = Object.keys(LANGUAGES) as LanguageCode[]

// 文档编辑链接
export const GITHUB_EDIT_URL = 'https://github.com/your-org/seed/tree/main/apps/docs'

// Algolia DocSearch 配置（需要申请）
export const ALGOLIA = {
  indexName: 'seed-docs',
  appId: 'XXXXXXXXXX',
  apiKey: 'XXXXXXXXXX',
}

// 侧边栏配置
export type SidebarItem = {
  text: string
  link?: string
  items?: SidebarItem[]
}

export type SidebarConfig = Record<LanguageCode, Record<string, SidebarItem[]>>

export const SIDEBAR: SidebarConfig = {
  'zh-cn': {
    快速开始: [
      { text: '介绍', link: '/zh-cn/introduction/' },
      { text: '快速开始', link: '/zh-cn/getting-started/' },
    ],
    开发指南: [
      { text: '项目结构', link: '/zh-cn/project-structure/' },
      { text: '前端开发', link: '/zh-cn/frontend/' },
      { text: '后端开发', link: '/zh-cn/backend/' },
    ],
    进阶: [
      { text: '架构设计', link: '/zh-cn/architecture/' },
      { text: '编码规范', link: '/zh-cn/conventions/' },
      { text: '部署指南', link: '/zh-cn/deployment/' },
    ],
  },
  en: {
    'Getting Started': [
      { text: 'Introduction', link: '/en/introduction/' },
      { text: 'Quick Start', link: '/en/getting-started/' },
    ],
    Development: [
      { text: 'Project Structure', link: '/en/project-structure/' },
      { text: 'Frontend', link: '/en/frontend/' },
      { text: 'Backend', link: '/en/backend/' },
    ],
    Advanced: [
      { text: 'Architecture', link: '/en/architecture/' },
      { text: 'Conventions', link: '/en/conventions/' },
      { text: 'Deployment', link: '/en/deployment/' },
    ],
  },
}

// 导航栏配置
export const NAV_LINKS = {
  'zh-cn': [
    { text: '文档', link: '/zh-cn/introduction/' },
    { text: 'GitHub', link: SITE.repo, external: true },
  ],
  en: [
    { text: 'Docs', link: '/en/introduction/' },
    { text: 'GitHub', link: SITE.repo, external: true },
  ],
}
