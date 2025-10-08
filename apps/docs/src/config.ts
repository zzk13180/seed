export const BASE = 'seed'

export const SITE = {
  title: 'Seed Docs',
  description: 'Documentation template for your project.',
  defaultLanguage: 'zh_CN',
}

// This is the type of the frontmatter you put in the docs markdown files.
export type Frontmatter = {
  title: string
  description: string
  layout: string
  image?: { src: string; alt: string }
  dir?: 'ltr' | 'rtl'
  ogLocale?: string
  lang?: string
}

export const KNOWN_LANGUAGES = {
  中文: 'zh_CN',
  English: 'en',
} as const
export const KNOWN_LANGUAGE_CODES = Object.values(KNOWN_LANGUAGES)

export const GITHUB_EDIT_URL = 'https://github.com/your-org/your-repo/tree/main/apps/docs'

// See "Algolia" section of the README for more information.
export const ALGOLIA = {
  indexName: 'XXXXXXXXXX',
  appId: 'XXXXXXXXXX',
  apiKey: 'XXXXXXXXXX',
}

export type Sidebar = Record<
  (typeof KNOWN_LANGUAGE_CODES)[number],
  Record<string, { text: string; link: string }[]>
>
export const SIDEBAR: Sidebar = {
  zh_CN: {
    快速开始: [
      { text: '介绍', link: 'zh_CN/introduction' },
      { text: '快速开始', link: 'zh_CN/getting-started' },
    ],
  },
  en: {
    'Getting Started': [
      { text: 'Introduction', link: 'en/introduction' },
      { text: 'Quick Start', link: 'en/getting-started' },
    ],
  },
}
