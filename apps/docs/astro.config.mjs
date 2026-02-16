import { defineConfig } from 'astro/config'
import preact from '@astrojs/preact'
import sitemap from '@astrojs/sitemap'
import tailwindcss from '@tailwindcss/vite'
import mdx from '@astrojs/mdx'
import icon from 'astro-icon'
import compressor from 'astro-compressor'

// Astro 5.x 最佳实践配置
export default defineConfig({
  site: 'https://zzk13180.github.io',
  base: '/seed',
  output: 'static',

  build: {
    format: 'directory', // 使用目录格式，更好的 URL 结构
    inlineStylesheets: 'auto',
  },

  trailingSlash: 'always',

  image: {
    domains: [],
    service: {
      entrypoint: 'astro/assets/services/sharp',
    },
  },

  // 修正 i18n 配置，与实际语言匹配
  i18n: {
    defaultLocale: 'zh-cn',
    locales: ['zh-cn', 'en'],
    fallback: {
      en: 'zh-cn',
    },
    routing: {
      prefixDefaultLocale: true, // 所有语言都带前缀，URL 更一致
    },
  },

  prefetch: {
    prefetchAll: false,
    defaultStrategy: 'hover',
  },

  vite: {
    plugins: [tailwindcss()],
    build: {
      cssMinify: 'lightningcss',
    },
  },

  markdown: {
    shikiConfig: {
      theme: 'github-dark',
      wrap: true,
    },
  },

  integrations: [
    mdx(),
    // 使用 Preact 处理所有交互组件（比 React 体积更小）
    preact({
      compat: true, // 启用 React 兼容，用于 Algolia 搜索等
    }),
    sitemap({
      i18n: {
        defaultLocale: 'zh-cn',
        locales: {
          'zh-cn': 'zh-CN',
          en: 'en-US',
        },
      },
    }),
    compressor({
      gzip: true,
      brotli: true,
    }),
    icon({
      include: {
        tabler: ['*'],
        lucide: ['*'],
        'flat-color-icons': [
          'template',
          'gallery',
          'approval',
          'document',
          'advertising',
          'currency-exchange',
          'voice-presentation',
          'business-contact',
          'database',
        ],
      },
    }),
  ],
})
