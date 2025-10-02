export const BASE = 'seed'

export const SITE = {
  title: 'Seed',
  description: 'monorepo seed project.',
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

export const GITHUB_EDIT_URL = 'https://github.com/zzk13180/seed/tree/main/apps/docs'

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
    Section: [
      { text: 'Introduction', link: 'zh_CN/introduction' },
      { text: 'Page 2', link: 'zh_CN/page-2' },
      { text: 'Page 3', link: 'zh_CN/page-3' },
      { text: 'Page 4', link: 'zh_CN/page-4' },
    ],
  },
  en: {
    Section: [
      { text: 'Introduction', link: 'en/introduction' },
      { text: 'Page 2', link: 'en/page-2' },
      { text: 'Page 3', link: 'en/page-3' },
      { text: 'Page 4', link: 'en/page-4' },
    ],
  },
}
