import { defineConfig } from 'astro/config'
import preact from '@astrojs/preact'
import react from '@astrojs/react'
import sitemap from '@astrojs/sitemap'
import tailwind from '@astrojs/tailwind'
import icon from 'astro-icon'
import compressor from 'astro-compressor'

export default defineConfig({
  site: 'https://zzk13180.github.io',
  base: '/seed',
  output: 'static',
  build: {
    format: 'file',
  },
  trailingSlash: 'ignore',
  image: {
    domains: [],
  },
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'fr'],
    fallback: {
      fr: 'en',
    },
    // routing: {
    //   prefixDefaultLocale: false,
    // },
  },
  prefetch: true,
  integrations: [
    // Enable Preact to support Preact JSX components.
    preact(),
    // Enable React for the Algolia search component.
    react(),
    tailwind({
      applyBaseStyles: false,
    }),
    sitemap(),
    compressor({
      gzip: true,
      brotli: true,
    }),
    icon({
      include: {
        tabler: ['*'],
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
