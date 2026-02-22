import { defineConfig } from 'astro/config'
import starlight from '@astrojs/starlight'

export default defineConfig({
  site: 'https://zzk13180.github.io',
  base: '/seed',
  integrations: [
    starlight({
      title: 'Seed',
      description: '企业级全栈多平台开发模板',
      defaultLocale: 'root',
      locales: {
        root: { label: '中文', lang: 'zh-CN' },
      },
      sidebar: [
        { label: '介绍', slug: 'index' },
        { label: '快速开始', slug: 'getting-started' },
        {
          label: '架构',
          items: [
            { label: '概述', slug: 'architecture' },
            { label: '后端', slug: 'architecture/backend' },
            { label: '前端', slug: 'architecture/frontend' },
            { label: '部署与安全', slug: 'architecture/deployment' },
          ],
        },
        { label: '代码约定', slug: 'conventions' },
        { label: '脚手架模板', slug: 'templates' },
      ],
      social: [
        {
          icon: 'github',
          label: 'GitHub',
          href: 'https://github.com/zzk13180/seed',
        },
      ],
      editLink: {
        baseUrl: 'https://github.com/zzk13180/seed/edit/main/apps/docs/',
      },
      lastUpdated: true,
    }),
  ],
})
